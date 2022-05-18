#include "trajectory_planner/trajectory_planner_plugin.hpp"

#include "geometry_msgs/msg/point32.hpp"
#include "nav2_util/costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2/convert.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <boost/crc.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filtering_stream.hpp>

#include <chrono>
#include <fstream>
#include <iomanip>
#include <limits>

using namespace std::chrono_literals;

namespace trajectory_planner
{

void TrajectoryPlannerPlugin::configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
    std::string name,
    std::shared_ptr<tf2_ros::Buffer>,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
    node_ = parent.lock();
    costmap_ros_ = costmap_ros;

    logger_ = node_->get_logger().get_child(name);

    RCLCPP_DEBUG((*logger_), "Trajectory Planner: configure()");

    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".angle_granularity", rclcpp::ParameterValue(128));
    node_->get_parameter(name + ".angle_granularity", angle_granularity_);
    if (angle_granularity_ < 0) {
        angle_granularity_ = 128;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'angle_granularity' value, using " << angle_granularity_);
    }

    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".inflation_radius", rclcpp::ParameterValue(2.0));
    node_->get_parameter(name + ".inflation_radius", inflation_radius_);
    if (inflation_radius_ <= 0.01) {
        inflation_radius_ = 2.0;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'inflation_radius' value, using " << inflation_radius_);
    }

    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".inflation_5d_radius", rclcpp::ParameterValue(0.2));
    node_->get_parameter(
        name + ".inflation_5d_radius",
        planning_parameters_.inflation_5d_radius);
    if (planning_parameters_.inflation_5d_radius < 0.01) {
        planning_parameters_.inflation_5d_radius = 0.01;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'inflation_5d_radius' value, using " <<
            planning_parameters_.inflation_5d_radius);
    }

    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".inflation_5d_lookahead", rclcpp::ParameterValue(0.5));
    node_->get_parameter(
        name + ".inflation_5d_lookahead",
        planning_parameters_.inflation_5d_lookahead);
    if (planning_parameters_.inflation_5d_lookahead < 0.01) {
        planning_parameters_.inflation_5d_lookahead = 0.01;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'inflation_5d_lookahead' value, using " <<
            planning_parameters_.inflation_5d_lookahead);
    }

    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".cache_directory", rclcpp::ParameterValue(""));
    node_->get_parameter(name + ".cache_directory", cache_directory_);

    nav2_util::declare_parameter_if_not_declared(
        node_, name + ".debug_directory", rclcpp::ParameterValue(""));
    node_->get_parameter(name + ".debug_directory", debug_directory_);

    RCLCPP_INFO_STREAM(
        (*logger_),
        "angle_granularity: " << angle_granularity_);
    RCLCPP_INFO_STREAM(
        (*logger_),
        "inflation_radius: " <<
        std::setprecision(3) << std::fixed <<
        inflation_radius_ << " m");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "inflation_5d_radius: " <<
        std::setprecision(3) << std::fixed <<
        planning_parameters_.inflation_5d_radius << " m");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "inflation_5d_lookahead: " <<
        std::setprecision(3) << std::fixed <<
        planning_parameters_.inflation_5d_lookahead << " m");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "cache_directory: '" << cache_directory_ << "'");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "debug_directory: '" << debug_directory_ << "'");

    pub_cost_map_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name + "/cost_map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_original_occupancy_map_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name + "/original_occupancy_map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_path_ =
        node_->create_publisher<nav_msgs::msg::Path>(
            "~/" + name + "/path",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_3d_debug_map_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name + "/three_dimension_planner_visited_space",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_5d_search_space_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name + "/five_dimension_planner_search_space",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_augmented_path_ =
        node_->create_publisher<trajectory_planner::msg::AugmentedPath>(
            "~/" + name + "/augmented_path",
            10);
}

std::string TrajectoryPlannerPlugin::hash_costmap(
    const Polygon& footprint,
    nav2_costmap_2d::Costmap2D* costmap) const
{
    // Convert a length value to millimeters and return it as an integer.
    auto to_mm = [](double d) {
        return static_cast<long int>(std::round(1000.0 * d));
    };

    boost::crc_32_type crc;

    // Include additional things that are not parameters to this function but
    // will influence the calculated costs of the planner.
    unsigned int inflation_radius = to_mm(inflation_radius_);
    crc.process_bytes(&inflation_radius, sizeof(inflation_radius));
    crc.process_bytes(&angle_granularity_, sizeof(angle_granularity_));

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

Polygon TrajectoryPlannerPlugin::convert_polygon_msg(
    const geometry_msgs::msg::Polygon& polygon) const
{
    Polygon result;

    std::transform(
        polygon.points.cbegin(),
        polygon.points.cend(),
        std::back_inserter(result),
        [](const geometry_msgs::msg::Point32& point) -> Point
        {
            return Point(point.x, point.y);
        });

    return result;
}

Pose TrajectoryPlannerPlugin::convert_pose_msg(
    const nav2_costmap_2d::Costmap2D* costmap,
    const geometry_msgs::msg::PoseStamped& pose_msg,
    unsigned int angle_granularity) const
{
    Pose result;

    if (!costmap->worldToMap(
        pose_msg.pose.position.x,
        pose_msg.pose.position.y,
        result.x,
        result.y)) {
        throw std::runtime_error("Unable to convert world coordinate to map.");
    }

    tf2::Quaternion q;
    tf2::fromMsg(pose_msg.pose.orientation, q);

    const tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    if (yaw < 0.0) {
        yaw += 2 * M_PI;
    }

    result.angle_index = static_cast<int>(std::round(yaw / (2 * M_PI / angle_granularity)));
    result.angle_index %= angle_granularity;

    return result;
}

geometry_msgs::msg::PoseStamped TrajectoryPlannerPlugin::convert_pose(
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& frame_id,
    const Pose& pose,
    unsigned int angle_granularity) const
{
    geometry_msgs::msg::PoseStamped result;

    result.header.frame_id = frame_id;

    costmap->mapToWorld(
        pose.x,
        pose.y,
        result.pose.position.x,
        result.pose.position.y);
    result.pose.position.z = 0.0;

    const double yaw = pose.angle_index * (2 * M_PI) / angle_granularity;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, yaw);

    result.pose.orientation = tf2::toMsg(q);

    return result;
}

Buffer<double> TrajectoryPlannerPlugin::create_occupancy_map(
    const nav2_costmap_2d::Costmap2D* costmap) const
{
    Buffer<double> result(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

    for (unsigned int y = 0; y < costmap->getSizeInCellsY(); y++) {
        for (unsigned int x = 0; x < costmap->getSizeInCellsX(); x++) {
            if (costmap->getCost(x, y) == nav2_util::Costmap::lethal_obstacle) {
                result.at(x, y) = 1.0;
            }
        }
    }

    return result;
}

nav_msgs::msg::OccupancyGrid TrajectoryPlannerPlugin::convert_buffer(
    const Buffer<double>& buffer,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y) const
{
    nav_msgs::msg::OccupancyGrid result;

    result.header.frame_id = frame_id;
    result.info.resolution = resolution;
    result.info.origin.position.x = origin_x;
    result.info.origin.position.y = origin_y;
    result.info.width = buffer.width();
    result.info.height = buffer.height();

    for (size_t y = 0; y < buffer.height(); y++) {
        for (size_t x = 0; x < buffer.width(); x++) {
            result.data.push_back(buffer.at(x, y) * 100);
        }
    }

    return result;
}

nav_msgs::msg::OccupancyGrid TrajectoryPlannerPlugin::convert_opened_nodes(
    const three::SearchResult3D::OpenedNodesMap& opened_nodes,
    size_t width,
    size_t height,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y) const
{
    nav_msgs::msg::OccupancyGrid result;

    result.header.frame_id = frame_id;
    result.info.resolution = resolution;
    result.info.origin.position.x = origin_x;
    result.info.origin.position.y = origin_y;
    result.info.width = width;
    result.info.height = height;

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            const auto it = opened_nodes.find(
                {static_cast<unsigned int>(x), static_cast<unsigned int>(y)});
            if (it == opened_nodes.end()) {
                // This coordinate was not visited.
                result.data.push_back(0);
            } else {
                // Add a negative value so that rviz colors this cell yellow-ish.
                result.data.push_back(-1 - it->second);
            }
        }
    }

    return result;
}

nav_msgs::msg::OccupancyGrid TrajectoryPlannerPlugin::convert_5d_search_space(
    const std::vector<five::SegmentSearchResult>& segments,
    size_t width,
    size_t height,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y) const
{
    nav_msgs::msg::OccupancyGrid result;

    result.header.frame_id = frame_id;
    result.info.resolution = resolution;
    result.info.origin.position.x = origin_x;
    result.info.origin.position.y = origin_y;
    result.info.width = width;
    result.info.height = height;

    // Combine the search spaces of all segments into a single set.
    std::unordered_set<Pose2D, boost::hash<Pose2D>> poses;
    for (const five::SegmentSearchResult& search_result: segments) {
        search_result.costs.export_poses(poses);
    }

    // "Draw" the search space on the map.
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            const Pose2D pose{static_cast<unsigned int>(x), static_cast<unsigned int>(y)};
            if (poses.contains(pose)) {
                // This cell is part of the search space, we color it yellow.
                result.data.push_back(-2);
            } else {
                result.data.push_back(0);
            }
        }
    }

    return result;
}

void TrajectoryPlannerPlugin::publish_augmented_path_messages(
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& frame_id,
    unsigned int angle_granularity,
    const Pose& start,
    const Pose& goal,
    const TrajectoryPlanner::Result result) const
{
    std::ostringstream ss;
    ss
        << std::setfill('0') << std::setw(5)
        << "path_"
        << start.x << "_" << start.y << "_" << start.angle_index
        << "_to_"
        << goal.x << "_" << goal.y << "_" << goal.angle_index;

    pub_augmented_path_->publish(
        create_augmented_path_message(
            ss.str() + "_3d",
            costmap,
            frame_id,
            angle_granularity,
            result.search_result_3d));
}

msg::AugmentedPath TrajectoryPlannerPlugin::create_augmented_path_message(
    const std::string& name,
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& frame_id,
    unsigned int angle_granularity,
    const three::SearchResult3D& search_result) const
{
    if (search_result.path.size() != search_result.heuristic.size()) {
        throw std::runtime_error("Search result heuristic vector has wrong size.");
    }
    if (search_result.path.size() != search_result.cost.size()) {
        throw std::runtime_error("Search result cost vector has wrong size.");
    }

    msg::AugmentedPath result;
    result.name = name;

    for (size_t i = 0; i < search_result.path.size(); ++i) {
        msg::AugmentedPose pose_message;

        pose_message.pose = convert_pose(
            costmap,
            frame_id,
            search_result.path.at(i),
            angle_granularity);

        pose_message.cost = search_result.cost.at(i);
        pose_message.heuristic = search_result.heuristic.at(i);

        result.poses.push_back(pose_message);
    }

    return result;
}

nav_msgs::msg::Path TrajectoryPlannerPlugin::convert_search_result(
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& costmap_frame_id,
    unsigned int angle_granularity,
    const TrajectoryPlanner::Result& result) const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = costmap_frame_id;

    // We create the final result by combining all the individual
    // segments that the 5D planner returned.

    for (const five::SegmentSearchResult& segment: result.search_result_5d.segment) {
        size_t segment_index = &segment - &(*result.search_result_5d.segment.begin());

        for (const Pose& pose: segment.path) {
            size_t pose_index = &pose - &(*segment.path.begin());

            if ((pose_index == 0) && (segment_index != 0)) {
                // We skip this one pose because it was already added as the last
                // pose of the previous segment.
                continue;
            }

            const geometry_msgs::msg::PoseStamped pose_msg = convert_pose(
                costmap, costmap_frame_id, pose, angle_granularity);

            path.poses.push_back(pose_msg);
        }
    }

    return path;
}

void TrajectoryPlannerPlugin::cleanup()
{
    RCLCPP_DEBUG((*logger_), "cleanup()");
}

void TrajectoryPlannerPlugin::activate()
{
    RCLCPP_DEBUG((*logger_), "activate()");

    pub_original_occupancy_map_->on_activate();
    pub_cost_map_->on_activate();
    pub_path_->on_activate();
    pub_3d_debug_map_->on_activate();
    pub_5d_search_space_->on_activate();
    pub_augmented_path_->on_activate();

    // (Re-)Start the thread that creates the planner and keeps it up-to-date.
    planner_update_thread_quit_ = false;
    planner_update_thread_ = std::thread(
        &TrajectoryPlannerPlugin::planner_update_thread_function, this);
}

void TrajectoryPlannerPlugin::deactivate()
{
    RCLCPP_DEBUG((*logger_), "deactivate()");

    pub_original_occupancy_map_->on_deactivate();
    pub_cost_map_->on_deactivate();
    pub_path_->on_deactivate();
    pub_3d_debug_map_->on_deactivate();
    pub_5d_search_space_->on_deactivate();
    pub_augmented_path_->on_deactivate();

    // Stop the background thread and wait for it to finish.
    std::unique_lock lock(planner_update_thread_mutex_);
    planner_update_thread_quit_ = true;
    lock.unlock();
    planner_update_thread_notification_.notify_all();
    planner_update_thread_.join();
}

nav_msgs::msg::Path TrajectoryPlannerPlugin::createPlan(
    const geometry_msgs::msg::PoseStamped& start_msg,
    const geometry_msgs::msg::PoseStamped& goal_msg)
{
    RCLCPP_DEBUG((*logger_), "createPlan()");

    nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
    const std::lock_guard<std::recursive_mutex> costmap_lock(*costmap->getMutex());

    const std::lock_guard<std::mutex> planner_lock(planner_mutex_);
    if (!planner_) {
        RCLCPP_DEBUG((*logger_), "Planner is not yet ready.");
        nav_msgs::msg::Path();
    }

    try {
        nav_msgs::msg::Path path_msg =
            plan(
                costmap,
                costmap_ros_->getBaseFrameID(),
                planner_.get(),
                start_msg,
                goal_msg);

        pub_path_->publish(path_msg);

        return path_msg;
    } catch (const std::exception& e) {
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "Error creating plan: " << e.what());
        return nav_msgs::msg::Path();
    }
}

nav_msgs::msg::Path TrajectoryPlannerPlugin::plan(
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& costmap_frame_id,
    const TrajectoryPlanner* planner,
    const geometry_msgs::msg::PoseStamped& start_msg,
    const geometry_msgs::msg::PoseStamped& goal_msg) const
{
    Pose start = convert_pose_msg(costmap, start_msg, planner_->angle_granularity());
    Pose goal = convert_pose_msg(costmap, goal_msg, planner_->angle_granularity());

    auto pose_to_string = [](const Pose& pose) {
        std::ostringstream ss;
        ss
            << pose.x << "/"
            << pose.y << "/"
            << pose.angle_index;
        return ss.str();
    };

    RCLCPP_INFO_STREAM(
        (*logger_),
        "Start: " << pose_to_string(start));
    RCLCPP_INFO_STREAM(
        (*logger_),
        "Goal:  " << pose_to_string(goal));

    auto timestamp_start = std::chrono::steady_clock::now();
    TrajectoryPlanner::Result result = planner->plan(planning_parameters_, start, goal);
    auto timestamp_end = std::chrono::steady_clock::now();

    std::chrono::duration<double> duration = timestamp_end - timestamp_start;
    RCLCPP_INFO_STREAM(
        (*logger_),
        "Planning took " << std::setprecision(3) << duration.count() << " sec.");

    RCLCPP_INFO_STREAM(
        (*logger_),
        "Search returned " << result.search_result_5d.segment.size() << " 5D segments:");
    for (const five::SegmentSearchResult& segment: result.search_result_5d.segment) {
        size_t index = &segment - &(*result.search_result_5d.segment.begin());
        RCLCPP_INFO_STREAM(
            (*logger_),
            "  segment " << index << " start: " << pose_to_string(*segment.path.begin()));
        RCLCPP_INFO_STREAM(
            (*logger_),
            "  segment " << index << " goal:  " << pose_to_string(*(segment.path.end() - 1)));
    }

    // Publish a map showing the nodes touched by the 3D planner.
    pub_3d_debug_map_->publish(
        convert_opened_nodes(
            result.search_result_3d.opened_nodes,
            planner->original_occupancy_map().width(),
            planner->original_occupancy_map().height(),
            costmap_frame_id,
            costmap->getResolution(),
            costmap->getOriginX(),
            costmap->getOriginY()));

    // Publish a map showing the search space used by the 5D planner.
    pub_5d_search_space_->publish(
        convert_5d_search_space(
            result.search_result_5d.segment,
            planner->original_occupancy_map().width(),
            planner->original_occupancy_map().height(),
            costmap_frame_id,
            costmap->getResolution(),
            costmap->getOriginX(),
            costmap->getOriginY()));

    publish_augmented_path_messages(
        costmap,
        costmap_frame_id,
        planner_->angle_granularity(),
        start,
        goal,
        result);

    // Convert the resulting path back to a ROS message.
    return convert_search_result(
        costmap,
        costmap_ros_->getGlobalFrameID(),
        planner_->angle_granularity(),
        result);
}

void TrajectoryPlannerPlugin::planner_update_thread_function()
{
    RCLCPP_INFO((*logger_), "Planner update thread started.");

    // Updating the planner in the overloaded functions of the plugin (for example
    // in 'configure()') proved to be problematic, for example at startup the
    // costmap can still be empty (and subscribing to the costmaps topics to

    for (;;) {
        std::unique_lock lock(planner_update_thread_mutex_);
        auto now = std::chrono::system_clock::now();
        if (planner_update_thread_notification_.wait_until(
                lock, now + 1s, [&](){ return planner_update_thread_quit_; })) {
            // The rest of the system wants us to stop.
            break;
        }

        //
        // Check if the costmap has changed and update the planner if necessary.
        //

        // Hash the current costmap.
        nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        const std::lock_guard<std::recursive_mutex> costmap_lock(*costmap->getMutex());
        Polygon footprint = convert_polygon_msg(costmap_ros_->getRobotFootprintPolygon());
        std::string new_hash = hash_costmap(footprint, costmap);

        // Get the old hash value, if there is currently a planner.
        std::string old_hash;
        {
            const std::lock_guard<std::mutex> planner_lock(planner_mutex_);
            if (planner_) {
                old_hash = planner_->hash();
            }
        }

        if (new_hash == old_hash) {
            // Nothing to do, sleep again.
            continue;
        }

        RCLCPP_INFO(
            (*logger_),
            "Costmap change detected, updating planner...");

        std::unique_ptr<TrajectoryPlanner> planner;

        std::chrono::time_point<std::chrono::steady_clock> timestamp_start;
        std::chrono::time_point<std::chrono::steady_clock> timestamp_end;
        std::chrono::duration<double> duration;

        auto log_callback = [this](const std::string msg) {
            RCLCPP_INFO((*this->logger_), msg.c_str());
        };

        bool planner_loaded = false;

        // Try to load the planner from a file.
        if (!cache_directory_.empty()) {
            std::ios_base::openmode mode = std::ios_base::in | std::ios_base::binary;
            std::ifstream file(cache_file_name(new_hash), mode);

            if (file.good()) {
                RCLCPP_INFO((*logger_), "Loading planner from cache...");

                boost::iostreams::filtering_istream ifs;
                ifs.push(boost::iostreams::zlib_decompressor());
                ifs.push(file);

                timestamp_start = std::chrono::steady_clock::now();
                planner = TrajectoryPlanner::load_planner(
                    log_callback,
                    costmap->getResolution(),
                    new_hash,
                    create_occupancy_map(costmap),
                    ifs);
                timestamp_end = std::chrono::steady_clock::now();

                if (planner) {
                    planner_loaded = true;

                    // Only log if loading was successful.
                    duration = timestamp_end - timestamp_start;
                    RCLCPP_INFO_STREAM(
                        (*logger_),
                        "Planner loaded in " <<
                        std::setprecision(3) << duration.count() << " sec.");
                }
            }
        }

        if (!planner) {
            // Either there was no planner to load or the loading failed,
            // re-create the whole planner.
            RCLCPP_INFO((*logger_), "Creating planner...");

            timestamp_start = std::chrono::steady_clock::now();
            planner = TrajectoryPlanner::create_planner(
                log_callback,
                true,
                new_hash,
                angle_granularity_,
                costmap->getResolution(),
                inflation_radius_,
                footprint,
                create_occupancy_map(costmap));
            timestamp_end = std::chrono::steady_clock::now();

            duration = timestamp_end - timestamp_start;
            RCLCPP_INFO_STREAM(
                (*logger_),
                "Planner created in " <<
                std::setprecision(3) << duration.count() << " sec.");
        }

        if (!planner_loaded && !cache_directory_.empty()) {
            RCLCPP_INFO((*logger_), "Writing planner to cache...");

            std::ios_base::openmode mode = std::ios_base::out | std::ios_base::binary;
            std::ofstream file(cache_file_name(new_hash), mode);

            if (!file.good()) {
                RCLCPP_ERROR_STREAM(
                    (*logger_),
                    "Unable to open cache file '" <<
                    cache_file_name(new_hash) <<
                    "' for writing.");
            } else {
                boost::iostreams::filtering_ostream ofs;
                ofs.push(boost::iostreams::zlib_compressor());
                ofs.push(file);

                timestamp_start = std::chrono::steady_clock::now();
                planner->store(ofs);
                timestamp_end = std::chrono::steady_clock::now();

                duration = timestamp_end - timestamp_start;
                RCLCPP_INFO_STREAM(
                    (*logger_),
                    "Planner written to '" <<
                    cache_file_name(new_hash) <<
                    "' in " <<
                    std::setprecision(3) << duration.count() << " sec.");
            }
        }

        pub_original_occupancy_map_->publish(
            convert_buffer(
                planner->original_occupancy_map(),
                costmap_ros_->getBaseFrameID(),
                costmap->getResolution(),
                costmap->getOriginX(),
                costmap->getOriginY()));
        pub_cost_map_->publish(
            convert_buffer(
                planner->cost_map(),
                costmap_ros_->getBaseFrameID(),
                costmap->getResolution(),
                costmap->getOriginX(),
                costmap->getOriginY()));

        if (!planner_loaded && !debug_directory_.empty()) {
            RCLCPP_INFO((*logger_), "Writing debug images...");

            timestamp_start = std::chrono::steady_clock::now();
            planner->dump_orientation_maps(debug_directory_ + "/orientation_");
            timestamp_end = std::chrono::steady_clock::now();

            duration = timestamp_end - timestamp_start;
            RCLCPP_INFO_STREAM(
                (*logger_),
                "Debug images written in " << std::setprecision(3) << duration.count() << " sec.");
        }

        const std::lock_guard<std::mutex> planner_lock(planner_mutex_);
        planner_ = std::move(planner);

        RCLCPP_INFO((*logger_), "Updated planner is ready.");
    }

    RCLCPP_INFO((*logger_), "Planner update thread done.");
}

std::string TrajectoryPlannerPlugin::cache_file_name(
    const std::string& hash) const
{
    return cache_directory_ + "/stored_trajectory_planner_" + hash + ".bin";
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    trajectory_planner::TrajectoryPlannerPlugin,
    nav2_core::GlobalPlanner)
