#include "trajectory_planner_plugin/convert.hpp"

namespace trajectory_planner::convert
{

#if ROS1
static const CostmapCostValue LETHAL_OBSTACLE = costmap_2d::LETHAL_OBSTACLE;
#else
static const CostmapCostValue LETHAL_OBSTACLE = nav2_util::Costmap::lethal_obstacle;
#endif

ROSTime ros_time(
    double seconds)
{
#if ROS1
    // ROS1: The ros::Time constructor takes the seconds directly.
    return ros::Time(seconds);
#else
    // ROS2: The rclcpp::Time constructor takes nanoseconds.
    return rclcpp::Time(seconds * 1e9);
#endif
}

Polygon polygon_msg_to_polygon(
    const ROSPolygon& polygon_msg)
{
    Polygon result;

    std::transform(
        polygon_msg.points.cbegin(),
        polygon_msg.points.cend(),
        std::back_inserter(result),
        [](const ROSPoint32& point) -> Point
        {
            return Point(point.x, point.y);
        });

    return result;
}

Buffer<double> create_occupancy_map(
    const Costmap2D* costmap)
{
    Buffer<double> result(costmap->getSizeInCellsX(), costmap->getSizeInCellsY());

    for (unsigned int y = 0; y < costmap->getSizeInCellsY(); y++) {
        for (unsigned int x = 0; x < costmap->getSizeInCellsX(); x++) {
            if (costmap->getCost(x, y) == LETHAL_OBSTACLE) {
                result.at(x, y) = 1.0;
            }
        }
    }

    return result;
}

ROSOccupancyGrid buffer_to_occupancy_grid_msg(
    const Buffer<double>& buffer,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y)
{
    ROSOccupancyGrid result;

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

Pose pose_msg_to_pose(
    const ROSPoseStamped& pose_msg,
    const Costmap2D* costmap,
    unsigned int angle_granularity)
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

ROSPoseStamped pose_to_pose_msg(
    const Costmap2D* costmap,
    const std::string& frame_id,
    double timestamp,
    const Pose& pose,
    unsigned int angle_granularity)
{
    ROSPoseStamped result;

    result.header.frame_id = frame_id;
    result.header.stamp = ros_time(timestamp);

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

ROSPoseStamped pose3d_to_pose_msg(
    const Costmap2D* costmap,
    const std::string& frame_id,
    double timestamp,
    const three::Pose3D& pose_3d)
{
    ROSPoseStamped result;

    result.header.frame_id = frame_id;
    result.header.stamp = ros_time(timestamp);

    costmap->mapToWorld(
        pose_3d.x,
        pose_3d.y,
        result.pose.position.x,
        result.pose.position.y);
    result.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, three::angle_lut[pose_3d.movement]);

    result.pose.orientation = tf2::toMsg(q);

    return result;
}

ROSPath plan_3d_to_path_msg(
    const Costmap2D* costmap,
    const std::string& costmap_frame_id,
    const TrajectoryPlanner::Result& result)
{
    ROSPath path;
    path.header.frame_id = costmap_frame_id;

    for (size_t i = 0; i < result.search_result_3d.path.size(); i++) {
        const ROSPoseStamped pose_msg = pose3d_to_pose_msg(
            costmap,
            costmap_frame_id,
            i * result.motion_model.time_delta(),
            result.search_result_3d.path.at(i));

        path.poses.push_back(pose_msg);
    }

    return path;
}

ROSOccupancyGrid heuristic_3d_to_occupancy_grid_msg(
    const DepthHeuristic& heuristic,
    size_t width,
    size_t height,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y)
{
    ROSOccupancyGrid result;

    result.header.frame_id = frame_id;
    result.info.resolution = resolution;
    result.info.origin.position.x = origin_x;
    result.info.origin.position.y = origin_y;
    result.info.width = width;
    result.info.height = height;

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            const float h = heuristic.get_value(x, y);
            if (std::isnan(h)) {
                // This cell is not part of the search space.
                result.data.push_back(0);
            } else {
                // Map the heuristic into the range from 1 to 98, which rviz
                // draws from blue to red in "costmap" mode.
                const float blue = 1.0f;
                const float red = 98.0f;

                // Small heuristic value (close to goal): blue
                // High heuristic value (far from goal): red
                const float color = blue + (h / heuristic.maximum_value() * (red - blue));

                result.data.push_back(static_cast<uint8_t>(color));
            }
        }
    }

    return result;
}

ROSOccupancyGrid opened_3d_nodes_to_occupancy_grid_msg(
    const three::SearchResult3D::OpenedNodesMap& opened_nodes,
    size_t width,
    size_t height,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y)
{
    ROSOccupancyGrid result;

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

ROSOccupancyGrid search_space_5d_to_occupacy_grid_msg(
    const std::vector<five::SegmentSearchResult>& segments,
    size_t width,
    size_t height,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y)
{
    ROSOccupancyGrid result;

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

    for (size_t i = segments.size(); i-- > 0;) {
        const five::SegmentSearchResult& segment = segments.at(i);

        if (segment.direction == five::Direction::TURN) {
            continue;
        }
        std::unordered_set<Pose2D, boost::hash<Pose2D>> poses;
        segment.costs.export_5d_poses(poses);

        for (const Pose2D& pose: poses) {
            const Pose5D dummy{pose.x, pose.y, 0, Pose5D::LinearVelocity(0), Pose5D::AngularVelocity(0)};
            const double h = segment.heuristic->value(dummy);

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

TPAugmentedPath search_result_3d_to_augmented_path_msg(
    const std::string& name,
    const Costmap2D* costmap_2d,
    const std::string& frame_id,
    const MotionModel& motion_model,
    const three::SearchResult3D& search_result)
{
    if (search_result.path.size() != search_result.path_heuristic.size()) {
        throw std::runtime_error("Search result heuristic vector has wrong size.");
    }
    if (search_result.path.size() != search_result.cost.size()) {
        throw std::runtime_error("Search result cost vector has wrong size.");
    }

    TPAugmentedPath result;
    result.name = name;
    result.motion_model = motion_model_to_motion_model_msg(motion_model);

    for (size_t i = 0; i < search_result.path.size(); ++i) {
        TPAugmentedPose pose_message;

        pose_message.pose = pose3d_to_pose_msg(
            costmap_2d,
            frame_id,
            0.0,
            search_result.path.at(i));

        pose_message.cost = search_result.cost.at(i);
        pose_message.heuristic = search_result.path_heuristic.at(i);

        result.poses.push_back(pose_message);
    }

    return result;
}

TPAugmentedPath search_result_5d_to_augmented_path_msg(
    const std::string& name,
    const Costmap2D* costmap_2d,
    const std::string& frame_id,
    unsigned int angle_granularity,
    const MotionModel& motion_model,
    const five::SegmentSearchResult& search_result)
{
    TPAugmentedPath result;
    result.name = name;
    result.motion_model = motion_model_to_motion_model_msg(motion_model);

    for (size_t i = 0; i < search_result.path.size(); ++i) {
        const Pose5D& pose_5d = search_result.path.at(i);
        const Pose pose{pose_5d.x, pose_5d.y, pose_5d.angle_index};

        TPAugmentedPose pose_message;

        pose_message.pose = pose_to_pose_msg(
            costmap_2d,
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

TPMotionModel motion_model_to_motion_model_msg(
    const MotionModel& motion_model)
{
    TPMotionModel result;

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

ROSPath search_result_5d_to_path_msg(
    const Costmap2D* costmap_2d,
    const std::string& costmap_frame_id,
    unsigned int angle_granularity,
    const TrajectoryPlanner::Result& result)
{
    ROSPath path;
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

            const ROSPoseStamped pose_msg = pose_to_pose_msg(
                costmap_2d,
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

}
