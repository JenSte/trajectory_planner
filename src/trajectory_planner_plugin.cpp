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
    name_ = name;

    logger_ = node_->get_logger().get_child(name);

    RCLCPP_DEBUG((*logger_), "Trajectory Planner: configure()");

    declare_parameters();


    pub_cost_map_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name + "/cost_map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_original_occupancy_map_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name + "/original_occupancy_map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_3d_path_ =
        node_->create_publisher<nav_msgs::msg::Path>(
            "~/" + name + "/three_dimension_planner_path",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_5d_path_ =
        node_->create_publisher<nav_msgs::msg::Path>(
            "~/" + name + "/five_dimension_planner_path",
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
    const double timestamp,
    const Pose& pose,
    const unsigned int angle_granularity) const
{
    geometry_msgs::msg::PoseStamped result;

    result.header.frame_id = frame_id;
    result.header.stamp = rclcpp::Time(timestamp * 1e9);

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

    // The biggest heuristic value we come across, used to scale down the rest.
    double maximum_heuristic_value = 1.0;

    // The poses in the search space, and their heuristic values.
    std::unordered_map<Pose2D, double, boost::hash<Pose2D>> heuristic_values;

    for (size_t i = segments.size() - 1; i < segments.size(); i--) {
        std::unordered_set<Pose2D, boost::hash<Pose2D>> poses;
        segments.at(i).costs.export_poses(poses);

        for (const Pose2D& pose: poses) {
            const Pose5D dummy{pose.x, pose.y, 0, Pose5D::LinearVelocity(0), Pose5D::AngularVelocity(0)};
            const double h = segments.at(i).heuristic->value(dummy);

            heuristic_values[pose] = h;
            maximum_heuristic_value = std::max(maximum_heuristic_value, h);
        }
    }

    // "Draw" the search space on the map.
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            const Pose2D pose{static_cast<unsigned int>(x), static_cast<unsigned int>(y)};

            const auto it = heuristic_values.find(pose);
            if (it == heuristic_values.end()) {
                // This cell is not part of the search space.
                result.data.push_back(0);
            } else {
                // Map the heuristic into the range from 0 to 98, which rviz
                // draws from blue to red in "costmap" mode.
                const uint8_t h = 1.0 + it->second / maximum_heuristic_value * 97.0;
                result.data.push_back(h);
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
    const TrajectoryPlanner::Result& result) const
{
    std::ostringstream ss;
    ss
        << std::setfill('0') << std::setw(5)
        << "path_"
        << start.x << "_" << start.y << "_" << start.angle_index
        << "_to_"
        << goal.x << "_" << goal.y << "_" << goal.angle_index;

    // One message for the 3D path.
    pub_augmented_path_->publish(
        create_augmented_path_message(
            ss.str() + "_3d",
            costmap,
            frame_id,
            angle_granularity,
            result.motion_model,
            result.search_result_3d));

    // Another message for every 5D path segment (that is not a turn segment).
    for (const five::SegmentSearchResult& result_5d: result.search_result_5d.segment) {
        if (result_5d.direction == five::Direction::TURN) {
            continue;
        }

        size_t index = &result_5d - &(*result.search_result_5d.segment.begin());

        std::ostringstream sss;
        sss
            << ss.str() + "_5d_segment_"
            << std::setfill('0') << std::setw(3) << index;

        pub_augmented_path_->publish(
            create_augmented_path_message(
                sss.str(),
                costmap,
                frame_id,
                angle_granularity,
                result.motion_model,
                result_5d));
    }
}

msg::AugmentedPath TrajectoryPlannerPlugin::create_augmented_path_message(
    const std::string& name,
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& frame_id,
    unsigned int angle_granularity,
    const MotionModel& motion_model,
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
    result.motion_model = create_motion_model_message(motion_model);

    for (size_t i = 0; i < search_result.path.size(); ++i) {
        msg::AugmentedPose pose_message;

        pose_message.pose = convert_pose(
            costmap,
            frame_id,
            0.0,
            search_result.path.at(i),
            angle_granularity);

        pose_message.cost = search_result.cost.at(i);
        pose_message.heuristic = search_result.heuristic.at(i);

        result.poses.push_back(pose_message);
    }

    return result;
}

msg::AugmentedPath TrajectoryPlannerPlugin::create_augmented_path_message(
    const std::string& name,
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& frame_id,
    unsigned int angle_granularity,
    const MotionModel& motion_model,
    const five::SegmentSearchResult& search_result) const
{
    msg::AugmentedPath result;
    result.name = name;
    result.motion_model = create_motion_model_message(motion_model);

    for (size_t i = 0; i < search_result.path.size(); ++i) {
        const Pose5D& pose_5d = search_result.path.at(i);
        const Pose pose{pose_5d.x, pose_5d.y, pose_5d.angle_index};

        msg::AugmentedPose pose_message;

        pose_message.pose = convert_pose(
            costmap,
            frame_id,
            i * motion_model.time_delta(),
            pose,
            angle_granularity);

        pose_message.linear_index = static_cast<int>(pose_5d.linear_velocity);
        pose_message.angular_index = static_cast<int>(pose_5d.angular_velocity);

        pose_message.cost = search_result.path_costs.at(i);
        pose_message.heuristic = search_result.path_heuristics.at(i);

        result.poses.push_back(pose_message);
    }

    return result;
}

msg::MotionModel TrajectoryPlannerPlugin::create_motion_model_message(
    const MotionModel& motion_model) const
{
    msg::MotionModel result;

    for (const double linear_velocity_value: motion_model.linear_steps()) {
        result.linear_velocities.push_back(linear_velocity_value);
    }

    for (const double angular_velocity_value: motion_model.angular_steps()) {
        result.angular_velocities.push_back(angular_velocity_value);
    }

    for (const MotionModel::KeyType& vels: motion_model.velocity_combinations()) {
        result.linear_indices.push_back(static_cast<int>(std::get<0>(vels)));
        result.angular_indices.push_back(static_cast<int>(std::get<1>(vels)));
    }

    return result;
}

nav_msgs::msg::Path TrajectoryPlannerPlugin::convert_3d_search_result(
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& costmap_frame_id,
    unsigned int angle_granularity,
    const TrajectoryPlanner::Result& result) const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = costmap_frame_id;

    for (size_t i = 0; i < result.search_result_3d.path.size(); i++) {
        const geometry_msgs::msg::PoseStamped pose_msg = convert_pose(
            costmap,
            costmap_frame_id,
            i * result.motion_model.time_delta(),
            result.search_result_3d.path.at(i),
            angle_granularity);

        path.poses.push_back(pose_msg);
    }

    return path;
}

nav_msgs::msg::Path TrajectoryPlannerPlugin::convert_5d_search_result(
    const nav2_costmap_2d::Costmap2D* costmap,
    const std::string& costmap_frame_id,
    unsigned int angle_granularity,
    const TrajectoryPlanner::Result& result) const
{
    nav_msgs::msg::Path path;
    path.header.frame_id = costmap_frame_id;

    // We create the final result by combining all the individual
    // segments that the 5D planner returned.

    size_t position = 0;

    for (const five::SegmentSearchResult& segment: result.search_result_5d.segment) {
        size_t segment_index = &segment - &(*result.search_result_5d.segment.begin());

        for (const Pose5D& pose: segment.path) {
            size_t pose_index = &pose - &(*segment.path.begin());

            if ((pose_index == 0) && (segment_index != 0)) {
                // We skip this one pose because it was already added as the last
                // pose of the previous segment.
                continue;
            }

            const geometry_msgs::msg::PoseStamped pose_msg = convert_pose(
                costmap,
                costmap_frame_id,
                position * result.motion_model.time_delta(),
                Pose{pose.x, pose.y, pose.angle_index},
                angle_granularity);

            position++;
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
    pub_3d_path_->on_activate();
    pub_5d_path_->on_activate();
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
    pub_3d_path_->on_deactivate();
    pub_5d_path_->on_deactivate();
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

        pub_5d_path_->publish(path_msg);

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

    auto pose_5d_to_string = [&pose_to_string](const Pose5D& pose) {
        return pose_to_string(Pose{pose.x, pose.y, pose.angle_index});
    };

    RCLCPP_INFO_STREAM(
        (*logger_),
        "Start: " << pose_to_string(start));
    RCLCPP_INFO_STREAM(
        (*logger_),
        "Goal:  " << pose_to_string(goal));

    TrajectoryPlanner::PlanningParameters planning_parameters = read_planning_parameters();

    auto timestamp_start = std::chrono::steady_clock::now();
    TrajectoryPlanner::Result result = planner->plan(planning_parameters, start, goal);
    auto timestamp_end = std::chrono::steady_clock::now();

    std::chrono::duration<double> duration = timestamp_end - timestamp_start;
    RCLCPP_INFO_STREAM(
        (*logger_),
        "Planning took " <<
        std::setprecision(3) << std::fixed <<
        duration.count() << " sec.");

    RCLCPP_INFO_STREAM(
        (*logger_),
        "Search returned " << result.search_result_5d.segment.size() << " 5D segment(s):");
    for (const five::SegmentSearchResult& segment: result.search_result_5d.segment) {
        size_t index = &segment - &(*result.search_result_5d.segment.begin());

        std::string direction = "turn";
        if (segment.direction == five::Direction::FORWARD) {
            direction = "forward";
        } else if (segment.direction == five::Direction::BACKWARD) {
            direction = "backward";
        }

        RCLCPP_INFO_STREAM(
            (*logger_),
            "  segment " << index << " direction: " << direction);
        if (segment.path.empty()) {
            RCLCPP_INFO_STREAM(
                (*logger_),
                "  segment " << index << " has no path elements");
        } else {
            RCLCPP_INFO_STREAM(
                (*logger_),
                "  segment " << index << " start: " << pose_5d_to_string(*segment.path.begin()));
            RCLCPP_INFO_STREAM(
                (*logger_),
                "  segment " << index << " goal:  " << pose_5d_to_string(*(segment.path.end() - 1)));
        }
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

    pub_3d_path_->publish(
        convert_3d_search_result(
            costmap,
            costmap_ros_->getGlobalFrameID(),
            planner_->angle_granularity(),
            result));

    publish_augmented_path_messages(
        costmap,
        costmap_frame_id,
        planner_->angle_granularity(),
        start,
        goal,
        result);

    // Convert the resulting path back to a ROS message.
    return convert_5d_search_result(
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
                        std::setprecision(3) << std::fixed <<
                        duration.count() << " sec.");
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
                std::setprecision(3) << std::fixed <<
                duration.count() << " sec.");
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
                    std::setprecision(3) << std::fixed <<
                    duration.count() << " sec.");
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
                "Debug images written in " <<
                std::setprecision(3) << std::fixed <<
                duration.count() << " sec.");
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

void TrajectoryPlannerPlugin::declare_parameters()
{
    // Parameters that influence the creation of the costmap. We are not able to
    // react to changes of them, so we mark them as read-only so that users are
    // not confused when changing them via rqt would not result in any change of
    // the cost map.

    rcl_interfaces::msg::ParameterDescriptor ro_descriptor;
    ro_descriptor.read_only = true;

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".angle_granularity", rclcpp::ParameterValue(128), ro_descriptor);
    node_->get_parameter(name_ + ".angle_granularity", angle_granularity_);
    if (angle_granularity_ < 0) {
        angle_granularity_ = 128;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'angle_granularity' value, using " << angle_granularity_);
    }

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".inflation_radius", rclcpp::ParameterValue(2.0), ro_descriptor);
    node_->get_parameter(name_ + ".inflation_radius", inflation_radius_);
    if (inflation_radius_ <= 0.01) {
        inflation_radius_ = 2.0;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'inflation_radius' value, using " << inflation_radius_);
    }

    // Unfortunately, rqt ignores the read only value for string parameters, even
    // when the correct type value is specified.
    ro_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".cache_directory", rclcpp::ParameterValue(""), ro_descriptor);
    node_->get_parameter(name_ + ".cache_directory", cache_directory_);

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".debug_directory", rclcpp::ParameterValue(""), ro_descriptor);
    node_->get_parameter(name_ + ".debug_directory", debug_directory_);

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
        "cache_directory: '" << cache_directory_ << "'");

    RCLCPP_INFO_STREAM(
        (*logger_),
        "debug_directory: '" << debug_directory_ << "'");

    // Parameters that only influence the planning (not the costmap creation). We
    // add ranges and descriptions for each of the parameters.

    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    descriptor.floating_point_range = {rcl_interfaces::msg::FloatingPointRange()};

    descriptor.description = "The maximum radius to inflate the 3D path, in meter.";
    descriptor.floating_point_range.at(0).from_value = 0.02;
    descriptor.floating_point_range.at(0).to_value = 0.5;
    descriptor.floating_point_range.at(0).step = 0.02;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".inflation_5d_radius", rclcpp::ParameterValue(0.2), descriptor);

    descriptor.description = "Lookahead distance when \"measuring\" the curvature of the 3D path, in meter";
    descriptor.floating_point_range.at(0).from_value = 0.02;
    descriptor.floating_point_range.at(0).to_value = 1.0;
    descriptor.floating_point_range.at(0).step = 0.02;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".inflation_5d_lookahead", rclcpp::ParameterValue(0.5), descriptor);

    descriptor.description = "The maximum linear velocity of the vehicle, in meter/second";
    descriptor.floating_point_range.at(0).from_value = 0.02;
    descriptor.floating_point_range.at(0).to_value = 2.0;
    descriptor.floating_point_range.at(0).step = 0.02;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".linear_velocity_maximum", rclcpp::ParameterValue(0.5), descriptor);

    descriptor.description = "The maximum angular velocity of the vehicle, in radian/second";
    descriptor.floating_point_range.at(0).from_value = 0.02;
    descriptor.floating_point_range.at(0).to_value = 2.0;
    descriptor.floating_point_range.at(0).step = 0.02;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".angular_velocity_maximum", rclcpp::ParameterValue(0.5), descriptor);

    descriptor.description = "The time increment used when simulating the motion, in seconds";
    descriptor.floating_point_range.at(0).from_value = 0.02;
    descriptor.floating_point_range.at(0).to_value = 1.0;
    descriptor.floating_point_range.at(0).step = 0.02;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".simulation_time_delta", rclcpp::ParameterValue(0.1), descriptor);

    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    descriptor.integer_range = {rcl_interfaces::msg::IntegerRange()};

    descriptor.description = "The number of steps to subdivide the maximum linear velocity into.";
    descriptor.integer_range.at(0).from_value = 1;
    descriptor.integer_range.at(0).to_value = 20;
    descriptor.integer_range.at(0).step = 1;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".linear_velocity_steps", rclcpp::ParameterValue(5), descriptor);

    descriptor.description = "The number of steps to subdivide the maximum angular velocity into.";
    descriptor.integer_range.at(0).from_value = 1;
    descriptor.integer_range.at(0).to_value = 20;
    descriptor.integer_range.at(0).step = 1;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".angular_velocity_steps", rclcpp::ParameterValue(4), descriptor);

    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;

    std::ostringstream ss;
    ss
        << "The heuristic to use for the 5D search ('"
        << HeuristicType::NONE << "'/'"
        << HeuristicType::EUCLIDEAN << "'/'"
        << HeuristicType::MANHATTAN << "'/'"
        << HeuristicType::DEPTH << "'/'"
        << HeuristicType::PATH << "').";
    descriptor.description = ss.str();
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".heuristic", rclcpp::ParameterValue("none"), descriptor);
}

TrajectoryPlanner::PlanningParameters TrajectoryPlannerPlugin::read_planning_parameters() const
{
    TrajectoryPlanner::PlanningParameters result;

    // Currently not used.
    result.linear_acceleration_maximum = 1.1;
    result.angular_acceleration_maximum = 3.5;

    result.multi_threaded = true;

    node_->get_parameter(name_ + ".inflation_5d_radius", result.inflation_5d_radius);
    if (result.inflation_5d_radius < 0.01) {
        result.inflation_5d_radius = 0.01;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'inflation_5d_radius' value, using " <<
            result.inflation_5d_radius);
    }

    node_->get_parameter(name_ + ".inflation_5d_lookahead", result.inflation_5d_lookahead);
    if (result.inflation_5d_lookahead < 0.01) {
        result.inflation_5d_lookahead = 0.01;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'inflation_5d_lookahead' value, using " <<
            result.inflation_5d_lookahead);
    }

    node_->get_parameter(name_ + ".linear_velocity_maximum", result.linear_velocity_maximum);
    if (result.linear_velocity_maximum < 0.01) {
        result.linear_velocity_maximum = 0.01;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'linear_velocity_maximum' value, using " <<
            result.linear_velocity_maximum);
    }

    node_->get_parameter(name_ + ".angular_velocity_maximum", result.angular_velocity_maximum);
    if (result.angular_velocity_maximum < 0.01) {
        result.angular_velocity_maximum = 0.01;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'angular_velocity_maximum' value, using " <<
            result.angular_velocity_maximum);
    }

    node_->get_parameter(name_ + ".simulation_time_delta", result.time_delta);
    if (result.time_delta < 0.01) {
        result.time_delta = 0.01;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'time_delta' value, using " <<
            result.time_delta);
    }

    int linear_velocity_steps;
    node_->get_parameter(name_ + ".linear_velocity_steps", linear_velocity_steps);
    if (linear_velocity_steps < 1) {
        linear_velocity_steps = 1;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'linear_velocity_steps' value, using " <<
            linear_velocity_steps);
    }
    result.linear_velocity_steps = linear_velocity_steps;

    int angular_velocity_steps;
    node_->get_parameter(name_ + ".angular_velocity_steps", angular_velocity_steps);
    if (angular_velocity_steps < 1) {
        angular_velocity_steps = 1;
        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'angular_velocity_steps' value, using " <<
            angular_velocity_steps);
    }
    result.angular_velocity_steps = angular_velocity_steps;

    std::string heuristic;
    node_->get_parameter(name_ + ".heuristic", heuristic);
    if (heuristic == "euclidean") {
        result.heuristic_type = HeuristicType::EUCLIDEAN;
    } else if (heuristic == "manhattan") {
        result.heuristic_type = HeuristicType::MANHATTAN;
    } else if (heuristic == "depth") {
        result.heuristic_type = HeuristicType::DEPTH;
    } else if (heuristic == "path") {
        result.heuristic_type = HeuristicType::PATH;
    } else {
        result.heuristic_type = HeuristicType::NONE;

        RCLCPP_ERROR_STREAM(
            (*logger_),
            "invalid 'heuristic' value, using '" <<
            result.heuristic_type <<
            "' (planning will be slow).");
    }

    RCLCPP_INFO((*logger_), "Planning parameters:");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  inflation_5d_radius: " <<
        std::setprecision(3) << std::fixed <<
        result.inflation_5d_radius << " m");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  inflation_5d_lookahead: " <<
        std::setprecision(3) << std::fixed <<
        result.inflation_5d_lookahead << " m");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  linear_velocity_maximum: " <<
        std::setprecision(3) << std::fixed <<
        result.linear_velocity_maximum << " m/s");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  angular_velocity_maximum: " <<
        std::setprecision(3) << std::fixed <<
        result.angular_velocity_maximum << " rad/s");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  linear_velocity_steps: " <<
        result.linear_velocity_steps);
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  angular_velocity_steps: " <<
        result.angular_velocity_steps);
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  linear_acceleration_maximum: " <<
        std::setprecision(3) << std::fixed <<
        result.linear_acceleration_maximum << " m/s^2");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  angular_acceleration_maximum: " <<
        std::setprecision(3) << std::fixed <<
        result.angular_acceleration_maximum << " rad/s^2");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  simulation_time_delta: " <<
        std::setprecision(3) << std::fixed <<
        result.time_delta << " s");
    RCLCPP_INFO_STREAM(
        (*logger_),
        "  heuristic: " <<
        result.heuristic_type);

    return result;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    trajectory_planner::TrajectoryPlannerPlugin,
    nav2_core::GlobalPlanner)
