#include "trajectory_planner_plugin/plugin.hpp"

namespace trajectory_planner
{

Plugin::Plugin()
    : costmap_observer_(
        [this](const std::string& message) {
            log_info(message);
        },
        [this](const convert::ROSOccupancyGrid& oc) {
            publish_original_occupancy_map(oc);
        },
        [this](const convert::ROSOccupancyGrid& oc) {
            publish_cost_map(oc);
        })
{
}

Plugin::~Plugin()
{
}

convert::ROSPath Plugin::create_plan(
    const convert::ROSPoseStamped& start_msg,
    const convert::ROSPoseStamped& goal_msg) const
{
    convert::Costmap2D* costmap_2d = costmap_ros_->getCostmap();
    convert::CostmapLockGuard costmap_lock(*(costmap_2d->getMutex()));

    std::shared_ptr<TrajectoryPlanner> planner = costmap_observer_.planner();
    if (!planner) {
        log_warn("Planner is not yet ready.");
        return convert::ROSPath();
    }

    try {
        TrajectoryPlanner::PlanningParameters pp = read_planning_parameters();
        cleanup_planning_parameters(pp);
        log_planning_parameters(pp);

        convert::ROSPath path_msg =
            plan(
                costmap_2d,
                planner.get(),
                start_msg,
                goal_msg,
                pp);

        return path_msg;
    } catch (const std::exception& e) {
        std::stringstream ss;
        ss << "Error creating plan: " << e.what();
        log_warn(ss.str());

        return convert::ROSPath();
    }
}

void Plugin::cleanup_planning_parameters(
    TrajectoryPlanner::PlanningParameters& parameters) const
{
    if (parameters.inflation_5d_radius < 0.01) {
        parameters.inflation_5d_radius = 0.01;
        std::ostringstream ss;
        ss
            << "invalid 'inflation_5d_radius' value, using "
            << std::setprecision(3) << std::fixed
            << parameters.inflation_5d_radius << " m";
        log_warn(ss.str());
    }

    if (parameters.inflation_5d_lookahead < 0.01) {
        parameters.inflation_5d_lookahead = 0.01;
        std::ostringstream ss;
        ss
            << "invalid 'inflation_5d_lookahead' value, using "
            << std::setprecision(3) << std::fixed
            << parameters.inflation_5d_lookahead << " m";
        log_warn(ss.str());
    }

    if (parameters.maximum_wheel_velocity < 0.01) {
        parameters.maximum_wheel_velocity = 0.01;
        std::ostringstream ss;
        ss
            << "invalid 'maximum_wheel_velocity' value, using "
            << std::setprecision(3) << std::fixed
            << parameters.maximum_wheel_velocity << " m/s";
        log_warn(ss.str());
    }

    if (parameters.maximum_wheel_acceleration < 0.01) {
        parameters.maximum_wheel_acceleration = 0.01;
        std::ostringstream ss;
        ss
            << "invalid 'maximum_wheel_acceleration' value, using "
            << std::setprecision(3) << std::fixed
            << parameters.maximum_wheel_acceleration << " m/s^2";
        log_warn(ss.str());
    }

    if (parameters.wheel_distance < 0.01) {
        parameters.wheel_distance = 0.01;
        std::ostringstream ss;
        ss
            << "invalid 'wheel_distance' value, using "
            << std::setprecision(2) << std::fixed
            << parameters.wheel_distance << " m";
        log_warn(ss.str());
    }

    if (parameters.linear_velocity_steps < 1) {
        parameters.linear_velocity_steps = 1;
        std::ostringstream ss;
        ss
            << "invalid 'linear_velocity_steps' value, using "
            << parameters.linear_velocity_steps;
        log_warn(ss.str());
    }

    if (parameters.angular_velocity_steps < 1) {
        parameters.angular_velocity_steps = 1;
        std::ostringstream ss;
        ss
            << "invalid 'angular_velocity_steps' value, using "
            << parameters.angular_velocity_steps;
        log_warn(ss.str());
    }
}

void Plugin::log_planning_parameters(
    const TrajectoryPlanner::PlanningParameters& parameters) const
{
    log_info("Planning parameters:");

    std::ostringstream ss;
    ss
        << "  inflation_5d_radius: "
        << std::setprecision(3) << std::fixed
        << parameters.inflation_5d_radius << " m";
    log_info(ss.str());

    ss.str("");
    ss
        << "  inflation_5d_lookahead: "
        << std::setprecision(3) << std::fixed
        << parameters.inflation_5d_lookahead << " m";
    log_info(ss.str());

    ss.str("");
    ss
        << "  maximum_wheel_velocity: "
        << std::setprecision(3) << std::fixed
        << parameters.maximum_wheel_velocity << " m/s";
    log_info(ss.str());

    ss.str("");
    ss
        << "  maximum_wheel_acceleration: "
        << std::setprecision(3) << std::fixed
        << parameters.maximum_wheel_acceleration << " m/s^2";
    log_info(ss.str());

    ss.str("");
    ss
        << "  wheel_distance: "
        << std::setprecision(2) << std::fixed
        << parameters.wheel_distance << " m";
    log_info(ss.str());

    ss.str("");
    ss
        << "  linear_velocity_steps: "
        << parameters.linear_velocity_steps;
    log_info(ss.str());

    ss.str("");
    ss
        << "  angular_velocity_steps: "
        << parameters.angular_velocity_steps;
    log_info(ss.str());

    ss.str("");
    ss
        << "  linear_acceleration_maximum: "
        << std::setprecision(3) << std::fixed
        << parameters.linear_acceleration_maximum << " m/s^2";
    log_info(ss.str());

    ss.str("");
    ss
        << "  angular_acceleration_maximum: "
        << std::setprecision(3) << std::fixed
        << parameters.angular_acceleration_maximum << " rad/s^2";
    log_info(ss.str());

    ss.str("");
    ss
        << "  heuristic: "
        << parameters.heuristic_type;
    log_info(ss.str());
}

convert::ROSPath Plugin::plan(
    const convert::Costmap2D* costmap_2d,
    const TrajectoryPlanner* planner,
    const convert::ROSPoseStamped& start_msg,
    const convert::ROSPoseStamped& goal_msg,
    const TrajectoryPlanner::PlanningParameters& planning_parameters) const
{
    Pose start = convert::pose_msg_to_pose(
        start_msg, costmap_2d, planner->angle_granularity());
    Pose goal = convert::pose_msg_to_pose(
        goal_msg, costmap_2d, planner->angle_granularity());

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

    log_info("Start pose: " + pose_to_string(start));
    log_info("Goal pose:  " + pose_to_string(goal));

    auto timestamp_start = std::chrono::steady_clock::now();
    TrajectoryPlanner::Result result = planner->plan(planning_parameters, start, goal);
    auto timestamp_end = std::chrono::steady_clock::now();

    std::chrono::duration<double> duration = timestamp_end - timestamp_start;
    std::stringstream ss;
    ss
        << "Planning took "
        << std::setprecision(3) << std::fixed
        << duration.count() << " sec.";
    log_info(ss.str());

    // Log some details on the 5D search result.

    ss.str("");
    ss
        << "Search returned "
        << result.search_result_5d.segment.size()
        << " 5D segment(s):";
    log_info(ss.str());

    for (const five::SegmentSearchResult& segment: result.search_result_5d.segment) {
        size_t index = &segment - &(*result.search_result_5d.segment.begin());

        ss.str("");
        ss << "  segment " << index << " direction: " << segment.direction;
        log_info(ss.str());

        if (segment.path.empty()) {
            ss.str("");
            ss << "  segment " << index << " has no path elements";
            log_info(ss.str());
        } else {
            ss.str("");
            ss << "  segment " << index << " start: " << pose_5d_to_string(*segment.path.begin());
            log_info(ss.str());

            ss.str("");
            ss << "  segment " << index << " goal:  " << pose_5d_to_string(*(segment.path.end() - 1));
            log_info(ss.str());
        }
    }

    // Publish the 3D plan.
    log_info("Publishing 3D plan...");
    publish_plan_3d(
        convert::plan_3d_to_path_msg(
            costmap_2d,
            costmap_ros_->getGlobalFrameID(),
            result));

    // Publish a map showing the heuristic used by the 3D planner.
    log_info("Publishing 3D heuristic map...");
    publish_heuristic_3d(
        convert::heuristic_3d_to_occupancy_grid_msg(
            result.search_result_3d.heuristic,
            planner->original_occupancy_map().width(),
            planner->original_occupancy_map().height(),
            costmap_ros_->getGlobalFrameID(),
            costmap_2d->getResolution(),
            costmap_2d->getOriginX(),
            costmap_2d->getOriginY()));

    // Publish a map showing the nodes opened by the 3D planner.
    if (planning_parameters.extract_3d_opened_nodes) {
        log_info("Publishing 3D opened nodes map...");
        publish_opened_3d_nodes_map(
            convert::opened_3d_nodes_to_occupancy_grid_msg(
                result.search_result_3d.opened_nodes,
                planner->original_occupancy_map().width(),
                planner->original_occupancy_map().height(),
                costmap_ros_->getGlobalFrameID(),
                costmap_2d->getResolution(),
                costmap_2d->getOriginX(),
                costmap_2d->getOriginY()));
    } else {
        log_info("Not publishing 3D opened nodes map (disabled by user).");
    }

    // Publish a map showing the search space used by the 5D planner.
    publish_search_space_5d_map(
        convert::search_space_5d_to_occupacy_grid_msg(
            result.search_result_5d.segment,
            planner->original_occupancy_map().width(),
            planner->original_occupancy_map().height(),
            costmap_ros_->getGlobalFrameID(),
            costmap_2d->getResolution(),
            costmap_2d->getOriginX(),
            costmap_2d->getOriginY()));

    // Publish additional path messages with more information than
    // what is included with the normal ROS messages.
    publish_augmented_path_messages(
        costmap_2d,
        costmap_ros_->getGlobalFrameID(),
        planner->angle_granularity(),
        start,
        goal,
        result);

    // Convert the resulting path back to a ROS message.
    convert::ROSPath result_msg = convert::search_result_5d_to_path_msg(
        costmap_2d,
        costmap_ros_->getGlobalFrameID(),
        planner->angle_granularity(),
        result);

    publish_plan_5d(result_msg);

    return result_msg;
}

void Plugin::publish_augmented_path_messages(
    const convert::Costmap2D* costmap,
    const std::string& frame_id,
    unsigned int angle_granularity,
    const Pose& start,
    const Pose& goal,
    const TrajectoryPlanner::Result& result) const
{
    if (result.search_result_3d.path.empty()) {
        return;
    }

    std::ostringstream ss;
    ss
        << std::setfill('0') << std::setw(5)
        << "path_"
        << start.x << "_" << start.y << "_" << start.angle_index
        << "_to_"
        << goal.x << "_" << goal.y << "_" << goal.angle_index;

    // One message for the 3D path.
    publish_augmented_path(
        convert::search_result_3d_to_augmented_path_msg(
            ss.str() + "_3d",
            costmap,
            frame_id,
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

        publish_augmented_path(
            convert::search_result_5d_to_augmented_path_msg(
                sss.str(),
                costmap,
                frame_id,
                angle_granularity,
                result.motion_model,
                result_5d));
    }
}

}
