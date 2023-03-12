#include "trajectory_planner_plugin/costmap_observer.hpp"

#include <boost/crc.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>

using namespace std::chrono_literals;

namespace trajectory_planner
{

CostmapObserver::CostmapObserver(
    LogCallback log_info,
    OccupancyGridCallback original_occupancy_map_callback,
    OccupancyGridCallback cost_map_callback)
    : log_info_(log_info)
    , original_occupancy_map_callback_(std::move(original_occupancy_map_callback))
    , cost_map_callback_(std::move(cost_map_callback))
{
}

CostmapObserver::~CostmapObserver()
{
    stop_watching();
}

std::shared_ptr<TrajectoryPlanner> CostmapObserver::planner() const
{
    return planner_;
}

void CostmapObserver::watch_costmap(
    int angle_granularity,
    double inflation_radius,
    const std::string& cache_directory,
    const std::string& debug_directory,
    std::optional<Point> internal_point,
    convert::CostmapROSPointer costmap)
{
    if (angle_granularity < 1) {
        angle_granularity = 128;

        std::ostringstream ss;
        ss
            << "invalid 'angle_granularity' value, using "
            << angle_granularity << " divisions";
        log_info_(ss.str());
    }

    if (inflation_radius <= 0.01) {
        inflation_radius = 2.0;

        std::ostringstream ss;
        ss
            << "invalid 'inflation_radius' value, using "
            << inflation_radius << " m";
        log_info_(ss.str());
    }

    // Log the parameters used to generate the planner.
    std::ostringstream ss;
    ss
        << "angle_granularity: "
        << angle_granularity << " divisions";
    log_info_(ss.str());

    ss.str("");
    ss
        << "inflation_radius: "
        << std::setprecision(3) << std::fixed
        << inflation_radius << " m";
    log_info_(ss.str());

    ss.str("");
    ss
        << "cache_directory: '"
        << cache_directory << "'";
    log_info_(ss.str());

    ss.str("");
    ss
        << "debug_directory: '"
        << debug_directory << "'";
    log_info_(ss.str());

    internal_point_ = internal_point;
    costmap_ros_ = costmap;

    planner_update_thread_quit_ = false;
    planner_update_thread_ = std::thread(
        &CostmapObserver::planner_update_thread_function,
        this,
        angle_granularity,
        inflation_radius,
        cache_directory,
        debug_directory);
}

void CostmapObserver::stop_watching()
{
    // Stop the background thread and wait for it to finish.
    {
        std::unique_lock lock(planner_update_thread_mutex_);
        if (planner_update_thread_quit_) {
            return;
        }

        planner_update_thread_quit_ = true;
    }

    planner_update_thread_notification_.notify_all();
    planner_update_thread_.join();
}

void CostmapObserver::planner_update_thread_function(
    int angle_granularity,
    double inflation_radius,
    std::string cache_directory,
    std::string debug_directory)
{
    log_info_("Planner update thread started.");

    // Updating the planner in the overloaded functions of the plugin (for example in
    // 'configure()') proved to be problematic, for example at startup the costmap can
    // still be empty (and subscribing to the costmaps topics to did not work at all
    // in ROS2 at all), and updating the planner takes some time. That's why it is
    // done here in a separate thread.

    for (;;) {
        {
            std::unique_lock lock(planner_update_thread_mutex_);
            auto now = std::chrono::system_clock::now();
            if (planner_update_thread_notification_.wait_until(
                    lock, now + 1s, [&](){ return planner_update_thread_quit_; })) {
                // The rest of the system wants us to stop.
                break;
            }
        }

        //
        // Check if the costmap has changed and update the planner if necessary.
        //

        // Hash the current costmap.
        convert::Costmap2D* costmap_2d = costmap_ros_->getCostmap();
        const convert::CostmapLockGuard costmap_lock(*(costmap_2d->getMutex()));
        Polygon footprint = convert::polygon_msg_to_polygon(
            costmap_ros_->getRobotFootprintPolygon());
        std::string new_hash = hash_costmap(
            angle_granularity, inflation_radius, footprint, costmap_2d);

        // Get the old hash value, if there is currently a planner.
        std::string old_hash;
        {
            if (planner_) {
                old_hash = planner_->hash();
            }
        }

        if (new_hash == old_hash) {
            // Nothing to do, sleep again.
            continue;
        }

        log_info_("Costmap change detected, updating planner...");

        std::unique_ptr<TrajectoryPlanner> planner;
        bool planner_loaded = false;

        // Try to load the planner from a file.
        if (!cache_directory.empty()) {
            planner = load_planner(
                cache_file_name(cache_directory, new_hash),
                new_hash,
                costmap_2d);

            if (planner) {
                // Remember that the planner was loaded from cache, so that
                // we can skip storing it and writing debug images down below.
                planner_loaded = true;
            }
        }

        if (!planner) {
            // Either there was no planner to load or the loading failed,
            // re-create the whole planner.
            planner = create_planner(
                new_hash,
                angle_granularity,
                inflation_radius,
                footprint,
                costmap_2d);
        }

        if (!planner_loaded && !cache_directory.empty()) {
            store_planner(cache_file_name(cache_directory, new_hash), planner.get());
        }

        if (original_occupancy_map_callback_) {
            original_occupancy_map_callback_(
                convert::buffer_to_occupancy_grid_msg(
                    planner->original_occupancy_map(),
                    costmap_ros_->getGlobalFrameID(),
                    costmap_2d->getResolution(),
                    costmap_2d->getOriginX(),
                    costmap_2d->getOriginY()));
        }

        if (cost_map_callback_) {
            cost_map_callback_(
                convert::buffer_to_occupancy_grid_msg(
                    planner->cost_map(),
                    costmap_ros_->getGlobalFrameID(),
                    costmap_2d->getResolution(),
                    costmap_2d->getOriginX(),
                    costmap_2d->getOriginY()));
        }

        if (!planner_loaded && !debug_directory.empty()) {
            write_debug_images(debug_directory, planner.get());
        }

        planner_ = std::move(planner);
        log_info_("Updated planner is ready.");
    }
}

std::string CostmapObserver::hash_costmap(
    int angle_granularity,
    double inflation_radius,
    const Polygon& footprint,
    const convert::Costmap2D* costmap) const
{
    // Convert a length value to millimeters and return it as an integer.
    auto to_mm = [](double d) {
        return static_cast<long int>(std::round(1000.0 * d));
    };

    boost::crc_32_type crc;

    // Include additional things that are not parameters to this function but
    // will influence the calculated costs of the planner.
    unsigned int inflation_radius_mm = to_mm(inflation_radius);
    crc.process_bytes(&inflation_radius_mm, sizeof(inflation_radius_mm));
    crc.process_bytes(&angle_granularity, sizeof(angle_granularity));

    for (const Point& point: footprint) {
        long int x = to_mm(std::get<0>(point));
        long int y = to_mm(std::get<1>(point));

        crc.process_bytes(&x, sizeof(x));
        crc.process_bytes(&y, sizeof(y));
    }

    unsigned int size_in_cells_x = costmap->getSizeInCellsX();
    unsigned int size_in_cells_y = costmap->getSizeInCellsY();
    long int origin_x = to_mm(costmap->getOriginX());
    long int origin_y = to_mm(costmap->getOriginY());
    long int resolution = to_mm(costmap->getResolution());

    crc.process_bytes(&size_in_cells_x, sizeof(size_in_cells_x));
    crc.process_bytes(&size_in_cells_y, sizeof(size_in_cells_y));
    crc.process_bytes(&origin_x, sizeof(origin_x));
    crc.process_bytes(&origin_y, sizeof(origin_y));
    crc.process_bytes(&resolution, sizeof(resolution));
    crc.process_bytes(
        costmap->getCharMap(),
        size_in_cells_x * size_in_cells_y * sizeof(unsigned char));

    std::stringstream ss;
    ss << std::hex << crc.checksum();
    return ss.str();
}

std::string CostmapObserver::cache_file_name(
    const std::string& cache_directory,
    const std::string& hash) const
{
    return cache_directory + "/stored_trajectory_planner_" + hash + ".bin";
}

std::unique_ptr<TrajectoryPlanner> CostmapObserver::load_planner(
    const std::string& cache_file,
    const std::string& hash,
    const convert::Costmap2D* costmap_2d) const
{
    auto log_callback = [this](const std::string& msg) {
        log_info_(msg);
    };

    std::chrono::time_point<std::chrono::steady_clock> timestamp_start;
    std::chrono::time_point<std::chrono::steady_clock> timestamp_end;
    std::chrono::duration<double> duration;

    std::ios_base::openmode mode = std::ios_base::in | std::ios_base::binary;
    std::ifstream file(cache_file, mode);

    if (!file.good()) {
        return nullptr;
    }

    log_info_("Loading planner from cache...");

    boost::iostreams::filtering_istream ifs;
    ifs.push(boost::iostreams::zlib_decompressor());
    ifs.push(file);

    timestamp_start = std::chrono::steady_clock::now();
    std::unique_ptr<TrajectoryPlanner> planner = TrajectoryPlanner::load_planner(
        log_callback,
        costmap_2d->getResolution(),
        hash,
        convert::create_occupancy_map(costmap_2d),
        ifs);
    timestamp_end = std::chrono::steady_clock::now();

    if (planner) {
        // Only log if loading was successful.
        duration = timestamp_end - timestamp_start;
        std::stringstream ss;
        ss
            << "Planner loaded in "
            << std::setprecision(3) << std::fixed
            << duration.count() << " sec.";
        log_info_(ss.str());
    }

    return planner;
}

std::unique_ptr<TrajectoryPlanner> CostmapObserver::create_planner(
    const std::string& hash,
    int angle_granularity,
    double inflation_radius,
    const Polygon& footprint,
    const convert::Costmap2D* costmap_2d) const
{
    log_info_("Creating planner...");

    auto log_callback = [this](const std::string& msg) {
        log_info_(msg);
    };

    std::chrono::time_point<std::chrono::steady_clock> timestamp_start;
    std::chrono::time_point<std::chrono::steady_clock> timestamp_end;
    std::chrono::duration<double> duration;

    std::optional<Pose2D> internal_point;
    if (internal_point_) {
        // If given, convert the coorindate of the internal point to pixels.
        unsigned int x, y;
        costmap_2d->worldToMap(
            std::get<0>(*internal_point_), std::get<1>(*internal_point_), x, y);
        internal_point = Pose2D{x, y};
    }

    timestamp_start = std::chrono::steady_clock::now();
    std::unique_ptr<TrajectoryPlanner> planner = TrajectoryPlanner::create_planner(
        log_callback,
        true,
        hash,
        angle_granularity,
        costmap_2d->getResolution(),
        inflation_radius,
        footprint,
        convert::create_occupancy_map(costmap_2d),
        internal_point);
    timestamp_end = std::chrono::steady_clock::now();

    duration = timestamp_end - timestamp_start;
    std::stringstream ss;
    ss
        << "Planner created in "
        << std::setprecision(3) << std::fixed
        << duration.count() << " sec.";
    log_info_(ss.str());

    return planner;
}

void CostmapObserver::store_planner(
    const std::string& cache_file,
    const TrajectoryPlanner* planner) const
{
    log_info_("Writing planner to cache...");

    std::chrono::time_point<std::chrono::steady_clock> timestamp_start;
    std::chrono::time_point<std::chrono::steady_clock> timestamp_end;
    std::chrono::duration<double> duration;

    std::ios_base::openmode mode = std::ios_base::out | std::ios_base::binary;
    std::ofstream file(cache_file, mode);

    if (!file.good()) {
        log_info_("Unable to open cache file '" + cache_file + "' for writing.");
        return;
    }

    boost::iostreams::filtering_ostream ofs;
    ofs.push(boost::iostreams::zlib_compressor());
    ofs.push(file);

    timestamp_start = std::chrono::steady_clock::now();
    planner->store(ofs);
    timestamp_end = std::chrono::steady_clock::now();

    duration = timestamp_end - timestamp_start;
    std::stringstream ss;
    ss
        << "Planner written to '"
        << cache_file
        << "' in "
        << std::setprecision(3) << std::fixed
        << duration.count() << " sec.";
    log_info_(ss.str());
}

void CostmapObserver::write_debug_images(
    const std::string& debug_directory,
    const TrajectoryPlanner* planner) const
{
    log_info_("Writing debug images...");

    std::chrono::time_point<std::chrono::steady_clock> timestamp_start;
    std::chrono::time_point<std::chrono::steady_clock> timestamp_end;
    std::chrono::duration<double> duration;

    timestamp_start = std::chrono::steady_clock::now();
    planner->dump_orientation_maps(debug_directory + "/orientation_");
    timestamp_end = std::chrono::steady_clock::now();

    duration = timestamp_end - timestamp_start;
    std::stringstream ss;
    ss
        << "Debug images written in "
        << std::setprecision(3) << std::fixed
        << duration.count() << " sec.";
    log_info_(ss.str());
}

}
