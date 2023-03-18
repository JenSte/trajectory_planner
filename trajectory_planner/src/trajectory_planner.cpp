#include "trajectory_planner/trajectory_planner.hpp"

#include "trajectory_planner/planning_3d.hpp"
#include "trajectory_planner/planning_5d.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <cmath>
#include <iomanip>

namespace trajectory_planner
{

TrajectoryPlanner::TrajectoryPlanner(
    LogCallback log_callback,
    double map_resolution,
    std::string hash,
    Buffer<double> occupancy_map,
    Buffer<double> cost_map,
    Costs costs)
    : log_callback_(std::move(log_callback))
    , map_resolution_(map_resolution)
    , hash_(std::move(hash))
    , original_occupancy_map_(std::move(occupancy_map))
    , cost_map_(std::move(cost_map))
    , costs_(std::move(costs))
{
}

std::unique_ptr<TrajectoryPlanner> TrajectoryPlanner::create_planner(
    LogCallback log_callback,
    const bool multi_threaded,
    std::string hash,
    unsigned int angle_granularity,
    double resolution,
    double inflation_radius,
    const Polygon& footprint,
    Buffer<double> occupancy_map,
    std::optional<Pose2D> internal_point)
{
    if (internal_point) {
        crop_outer(log_callback, *internal_point, occupancy_map);
    }

    Buffer<double> cost_map = create_cost_map(
        log_callback,
        multi_threaded,
        resolution,
        inflation_radius,
        occupancy_map);
    Costs costs = create_costs(
        log_callback,
        multi_threaded,
        angle_granularity,
        resolution,
        footprint,
        occupancy_map,
        cost_map);

    std::unique_ptr<TrajectoryPlanner> planner;
    planner.reset(new TrajectoryPlanner(
        std::move(log_callback),
        resolution,
        std::move(hash),
        std::move(occupancy_map),
        std::move(cost_map),
        std::move(costs)));
    return planner;
}

std::unique_ptr<TrajectoryPlanner> TrajectoryPlanner::load_planner(
    LogCallback log_callback,
    double resolution,
    std::string hash,
    Buffer<double> occupancy_map,
    std::istream& istream)
{
    // Create a dummy cost map when loading the planner from a file,
    // as the cost map is only used for debugging purposes.
    Buffer<double> cost_map(occupancy_map.width(), occupancy_map.height());
    draw_text(cost_map, "Cost map not available for planners loaded from cache.");

    Costs costs(0, 0, 0, 1);
    try {
        // Load the costs object.
        boost::archive::binary_iarchive ia(istream);
        ia >> costs;
    } catch (const std::exception& e) {
        log_callback(std::string("Error loading planner: ") + e.what());
        return nullptr;
    }

    std::unique_ptr<TrajectoryPlanner> planner;
    planner.reset(new TrajectoryPlanner(
        std::move(log_callback),
        resolution,
        std::move(hash),
        std::move(occupancy_map),
        std::move(cost_map),
        std::move(costs)));
    return planner;
}

void TrajectoryPlanner::crop_outer(
    const LogCallback& log_callback,
    Pose2D point,
    Buffer<double>& map)
{
    if (point.x >= map.width()) {
        std::ostringstream ss;
        ss
            << "X-coordinate of internal point (" << point.x
            << ") too huge for a map of width " << map.width()
            << ", not cropping out the internal part.";
        log_callback(ss.str());
        return;
    }

    if (point.y >= map.height()) {
        std::ostringstream ss;
        ss
            << "Y-coordinate of internal point (" << point.y
            << ") too huge for a map of height " << map.height()
            << ", not cropping out the internal part.";
        log_callback(ss.str());
        return;
    }

    std::ostringstream ss;
    ss
        << "Cropping out internal space around pixel "
        << point.x << "/" << point.y << "...";
    log_callback(ss.str());

    using set = std::unordered_set<Pose2D, boost::hash<Pose2D>>;

    // Insert a given point 'p' into the set 's' if it is a valid
    // coordinate on the map 'map' and the point is free.
    auto insert_maybe = [&map](set& s, Pose2D p) {
        if (p.x >= map.width()) {
            return;
        }

        if (p.y >= map.height()) {
            return;
        }

        const double pixel = map.at(p.x, p.y);
        if (pixel > 0.5) {
            return;
        }

        // Point is on the map and free.
        s.insert(p);
    };

    // The pixels that are connected to 'point' with free space.
    set internal;

    // The pixels that are currently processed.
    set frontier;

    // The checks at the beginning of this function already made sure that the
    // coordinates of 'point' are valid, but we only add it to the start set
    // if it is actually unoccupied.
    if (map.at(point.x, point.y) < 0.5) {
        internal.insert(point);
        frontier.insert(point);
    } else {
        log_callback("Given internal pixel is occupied!");
    }

    while (!frontier.empty()) {
        // The candidates for the new frontier set when we go forward, all
        // the neighbours of the current frontier. In this flood-fill implementation,
        // we don't go diagonal, so that we do not "enter" spaces where only a
        // free diagonal neighbour exists.
        set candidates;
        for (const Pose2D& f: frontier) {
            // Go up/down/left/right for every pixel in 'frontier'.
            insert_maybe(candidates, {f.x + 1, f.y});
            insert_maybe(candidates, {f.x - 1, f.y});
            insert_maybe(candidates, {f.x, f.y + 1});
            insert_maybe(candidates, {f.x, f.y - 1});
        }

        frontier.clear();
        for (const Pose2D& c: candidates) {
            const auto it = internal.find(c);
            if (it == internal.end()) {
                frontier.insert(c);
                internal.insert(c);
            }
        }
    }

    if (!internal.empty()) {
        std::ostringstream ss;
        ss
            << "Interal part of map contains " << internal.size() << " pixels.";
        log_callback(ss.str());
    } else {
        log_callback("Map is empty after attempting to crop out inner part.");
    }

    for (unsigned int x = 0; x < map.width(); x++) {
        for (unsigned int y = 0; y < map.height(); y++) {
            const auto it = internal.find({x, y});
            if (it == internal.end()) {
                // This pixel is not in the 'internal' set, so paint it black.
                map.at(x, y) = 1.0;
            }
        }
    }
}

void TrajectoryPlanner::draw_text(
    Buffer<double>& buffer,
    const std::string& text)
{
    const int face = cv::FONT_HERSHEY_PLAIN;
    const double scale = 1.0;
    const int thickness = 1;

    int baseline = 0;
    cv::Size size = cv::getTextSize(text, face, scale, thickness, &baseline);
    size += cv::Size(0, baseline);
    const cv::Point org(0, size.height + 2 - baseline);

    const cv::Scalar color(255);
    cv::Mat canvas(cv::Mat::zeros(size, CV_8UC1));
    cv::putText(canvas, text, org, face, scale, color, thickness);
    cv::line(canvas, org + cv::Point(0, thickness), org + cv::Point(size.width, thickness), color);

    for (size_t column = 0; column < static_cast<size_t>(canvas.cols); column++) {
        for (size_t row = 0; row < static_cast<size_t>(canvas.rows); row++) {
            const bool pixel = canvas.at<unsigned char>(row, column) != 0;

            {
                // Copy text horizontally.
                const size_t x = size.height + column;
                const size_t y = canvas.rows - 1 - row;

                if ((x < buffer.width()) && (y < buffer.height())) {
                    buffer.at(x, y) = pixel ? 1.0 : 0.0;
                }
            }

            {
                // Copy text vertically.
                const size_t x = row;
                const size_t y = size.height + column;

                if ((x < buffer.width()) && (y < buffer.height())) {
                    buffer.at(x, y) = pixel ? 1.0 : 0.0;
                }
            }
        }
    }
}


const Buffer<double>& TrajectoryPlanner::original_occupancy_map() const
{
    return original_occupancy_map_;
}

const Buffer<double>& TrajectoryPlanner::cost_map() const
{
    return cost_map_;
}

const std::string& TrajectoryPlanner::hash() const
{
    return hash_;
}

unsigned int TrajectoryPlanner::angle_granularity() const
{
    return costs_.angle_granularity();
}

TrajectoryPlanner::Result TrajectoryPlanner::plan(
    const PlanningParameters& parameters,
    const Pose& start,
    const Pose& goal) const
{
    check_pose(start, "start");
    check_pose(goal, "goal");

    // This object is used to do the simulation of the 5D movement.
    MotionModel motion_model(
        parameters.maximum_wheel_velocity,
        parameters.maximum_wheel_acceleration,
        parameters.wheel_distance,
//        map_resolution_,
//        costs_.angle_granularity(),
        parameters.linear_velocity_steps,
        parameters.angular_velocity_steps);

    log_motion_model(motion_model);

    // Plan in three dimensions.
    auto timestamp_start = std::chrono::steady_clock::now();
    three::SearchResult3D result_3d = three::plan(costs_, start, goal);
    auto timestamp_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = timestamp_end - timestamp_start;

    std::ostringstream ss;
    ss
        << "3D search finished in "
        << std::setprecision(3) << std::fixed
        << duration.count() << " sec.";
    log_callback_(ss.str());

    // Plan in five dimensions.
    timestamp_start = std::chrono::steady_clock::now();
    five::SearchResult5D result_5d = five::plan(
        parameters.multi_threaded,
        map_resolution_,
        parameters.inflation_5d_radius / map_resolution_,
        parameters.inflation_5d_lookahead / map_resolution_,
        parameters.heuristic_type,
        costs_,
        motion_model,
        result_3d.path);
    timestamp_end = std::chrono::steady_clock::now();
    duration = timestamp_end - timestamp_start;

    ss.str("");
    ss
        << "5D search finished in "
        << std::setprecision(3) << std::fixed
        << duration.count() << " sec.";
    log_callback_(ss.str());

    return Result{std::move(result_3d), std::move(result_5d), std::move(motion_model)};
}

void TrajectoryPlanner::store(
    std::ostream& ostream) const
{
    boost::archive::binary_oarchive oa(ostream);
    oa << costs_;
}

void TrajectoryPlanner::check_pose(
    const Pose& pose,
    const std::string& pose_name) const
{
    if (!(pose.x < costs_.width())) {
        std::ostringstream ss;
        ss
            << "X coordinate (" << pose.x << ") of the " << pose_name
            << " pose exceeds map's width (" << costs_.width() << ").";
        throw std::invalid_argument(ss.str());
    }

    if (!(pose.y < costs_.height())) {
        std::ostringstream ss;
        ss
            << "Y coordinate (" << pose.y << ") of the " << pose_name
            << " pose exceeds map's height (" << costs_.height() << ").";
        throw std::invalid_argument(ss.str());
    }

    if (!costs_.get_5d_cost(pose.x, pose.y, pose.angle_index)) {
        throw std::invalid_argument("The " + pose_name + " pose is not valid.");
    }
}

unsigned int TrajectoryPlanner::calculate_footprint_size(
    const Polygon& footprint_polygon,
    double resolution)
{
    int size = 2;

    for (const auto& p: footprint_polygon) {
        // The distance of the point from the center.
        double dist_meter =
            sqrt(pow(std::get<0>(p), 2.0) + pow(std::get<1>(p), 2.0));

        // Multiply the distance with a value that is a little bit bigger
        // than sqrt(2), so that even if the footprint is rotated by
        // 45 degrees it will still fit.
        dist_meter *= 1.5;

        // The distance in pixels.
        int dist_pixels = dist_meter / resolution;

        size = std::max(size, 2 * dist_pixels);
    }

    return size;
}

void TrajectoryPlanner::log_motion_model(
    const MotionModel& motion_model) const
{
    log_callback_("motion model:");

    std::ostringstream ss;

    ss << "  linear steps (m/s):   " << std::setprecision(2) << std::fixed;
    for (double vel: motion_model.linear_steps()) {
        ss << " " << vel;
    }
    log_callback_(ss.str());

    ss.str("");
    ss << "  angular steps (rad/s):" << std::setprecision(2) << std::fixed;
    for (double vel: motion_model.angular_steps()) {
        ss << " " << vel;
    }
    log_callback_(ss.str());

    ss.str("");
    ss
        << "  linear acceleration between steps:  "
        << std::setprecision(3) << std::fixed
        << (motion_model.linear_steps().at(1) / motion_model.time_delta())
        << " m/s^2";
    log_callback_(ss.str());

    ss.str("");
    ss
        << "  angular acceleration between steps: "
        << std::setprecision(3) << std::fixed
        << (motion_model.angular_steps().at(1) / motion_model.time_delta())
        << " rad/s^2";
    log_callback_(ss.str());

    ss.str("");
    ss
        << "  time step: "
        << std::setprecision(3) << std::fixed
        << motion_model.time_delta()
        << " s";
    log_callback_(ss.str());

    // The smallest distance and angle that can be traversed with the smallest
    // linear/angular velocities in one simulation step.
    double smallest_distance = motion_model.linear_steps().at(1) * motion_model.time_delta();
    double smallest_angle = motion_model.angular_steps().at(1) * motion_model.time_delta();

    ss.str("");
    ss
        << "  smallest travelled distance: "
        << std::setprecision(3) << std::fixed
        << smallest_distance
        << " m";
    log_callback_(ss.str());

    // Warn if the smallest travelled distance does not make much progress on the
    // given map.
    if (smallest_distance < (0.5 * sqrt(2.0) * map_resolution_)) {
        log_callback_("  warning: smallest travelled distance seems small for the map resolution");
    }

    ss.str("");
    ss
        << "  smallest travelled angle: "
        << std::setprecision(3) << std::fixed
        << smallest_angle
        << " rad";
    log_callback_(ss.str());

    // Calculate the ideal angle granularity for the smallest possible angle.
    int ideal_angle_granularity = 2 * M_PI / smallest_angle;

    ss.str("");
    ss
        << "  ideal angle granularity: "
        << ideal_angle_granularity;
    log_callback_(ss.str());

    // Warn if the angle granularity is off.
    if (abs(ideal_angle_granularity - static_cast<int>(costs_.angle_granularity())) > 5) {
        log_callback_("  warning: angle_granularity is not ideal");
    }
}

}
