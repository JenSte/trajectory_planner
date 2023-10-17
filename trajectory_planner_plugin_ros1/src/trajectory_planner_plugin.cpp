#include "trajectory_planner_plugin/trajectory_planner_plugin.hpp"

namespace trajectory_planner
{

TrajectoryPlannerPlugin::TrajectoryPlannerPlugin()
{
    ROS_INFO("TrajectoryPlannerPlugin created");
}

void TrajectoryPlannerPlugin::initialize(
    std::string name,
    costmap_2d::Costmap2DROS* costmap_ros)
{
    create_publishers(name);

    int angle_granularity;
    double inscribed_radius;
    double inflation_radius;
    std::string cache_directory;
    std::string debug_directory;
    trajectory_planner::three::CostMap3DType cost_map_3d_type;
    trajectory_planner::three::OrientationCosts3DType orientation_costs_3d_type;

    nh_.getParam("angle_granularity", angle_granularity);
    nh_.getParam("inscribed_radius", inscribed_radius);
    nh_.getParam("inflation_radius", inflation_radius);
    nh_.getParam("cache_directory", cache_directory);
    nh_.getParam("debug_directory", debug_directory);

    std::string str;
    nh_.getParam("cost_map_3d_type", str);
    if (str == "linear") {
        cost_map_3d_type = trajectory_planner::three::CostMap3DType::LINEAR;
    } else if (str == "exponential") {
        cost_map_3d_type = trajectory_planner::three::CostMap3DType::EXPONENTIAL;
    } else {
        cost_map_3d_type = trajectory_planner::three::CostMap3DType::EXPONENTIAL;
        ROS_WARN("Unable to parse cost_map_3d_type, using 'exponental'." );
    }

    nh_.getParam("orientation_costs_3d_type", str);
    if (str == "identical") {
        orientation_costs_3d_type = trajectory_planner::three::OrientationCosts3DType::IDENTICAL;
    } else if (str == "footprint") {
        orientation_costs_3d_type = trajectory_planner::three::OrientationCosts3DType::FOOTPRINT;
    } else {
        orientation_costs_3d_type = trajectory_planner::three::OrientationCosts3DType::IDENTICAL;
        ROS_WARN("Unable to parse orientation_costs_3d_type, using 'identical'.");
    }

    std::optional<Point> internal_point;
    if (nh_.hasParam("internal_point_x") && nh_.hasParam("internal_point_y")) {
        double x, y;
        nh_.getParam("internal_point_x", x);
        nh_.getParam("internal_point_y", y);

        internal_point = Point(x, y);
    }

    costmap_ros_ = costmap_ros;

    costmap_observer_.watch_costmap(
        cost_map_3d_type,
        orientation_costs_3d_type,
        angle_granularity,
        inscribed_radius,
        inflation_radius,
        cache_directory,
        debug_directory,
        internal_point,
        costmap_ros_);
}

bool TrajectoryPlannerPlugin::makePlan(
    const geometry_msgs::PoseStamped& start_msg,
    const geometry_msgs::PoseStamped& goal_msg,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
    nav_msgs::Path path = create_plan(start_msg, goal_msg);

    std::copy(
        path.poses.cbegin(),
        path.poses.cend(),
        std::back_inserter(plan));

    ros::Time now = ros::Time::now();
    for (geometry_msgs::PoseStamped& pose: plan) {
        pose.header.stamp = now + ros::Duration(pose.header.stamp.sec, pose.header.stamp.nsec);
    }

    return !path.poses.empty();
}

void TrajectoryPlannerPlugin::log_info(
    const std::string& message) const
{
    ROS_INFO_STREAM(message);
}

void TrajectoryPlannerPlugin::log_warn(
    const std::string& message) const
{
    ROS_WARN_STREAM(message);
}

TrajectoryPlanner::PlanningParameters TrajectoryPlannerPlugin::read_planning_parameters() const
{
    TrajectoryPlanner::PlanningParameters result;

    // Currently not used.
    result.linear_acceleration_maximum = 1.1;
    result.angular_acceleration_maximum = 3.5;

    result.multi_threaded = true;

    nh_.getParam("inflation_5d_radius", result.inflation_5d_radius);
    nh_.getParam("inflation_5d_lookahead", result.inflation_5d_lookahead);
    nh_.getParam("maximum_wheel_velocity", result.maximum_wheel_velocity);
    nh_.getParam("maximum_wheel_acceleration", result.maximum_wheel_acceleration);
    nh_.getParam("wheel_distance", result.wheel_distance);
    //nh_.getParam("linear_velocity_maximum", result.linear_velocity_maximum);
    //nh_.getParam("angular_velocity_maximum", result.angular_velocity_maximum);
    //nh_.getParam("simulation_time_delta", result.time_delta);

    int linear_velocity_steps;
    nh_.getParam("linear_velocity_steps", linear_velocity_steps);
    result.linear_velocity_steps = linear_velocity_steps;

    int angular_velocity_steps;
    nh_.getParam("angular_velocity_steps", angular_velocity_steps);
    result.angular_velocity_steps = angular_velocity_steps;

    std::string heuristic;
    nh_.getParam("heuristic", heuristic);
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

        ROS_ERROR_STREAM(
            "invalid 'heuristic' value, using '" <<
            result.heuristic_type <<
            "' (planning will be slow).");
    }

    result.extract_3d_opened_nodes = false;
    nh_.getParam("extract_3d_opened_nodes", result.extract_3d_opened_nodes);

    return result;
}

void TrajectoryPlannerPlugin::publish_original_occupancy_map(
    const nav_msgs::OccupancyGrid& msg) const
{
    pub_original_occupancy_map_.publish(msg);
}

void TrajectoryPlannerPlugin::publish_cost_map(
    const nav_msgs::OccupancyGrid& msg) const
{
    pub_cost_map_.publish(msg);
}

void TrajectoryPlannerPlugin::publish_plan_3d(
    const convert::ROSPath& msg) const
{
    pub_plan_3d_.publish(msg);
}

void TrajectoryPlannerPlugin::publish_heuristic_3d(
    const convert::ROSOccupancyGrid& msg) const
{
    pub_heuristic_3d_.publish(msg);
}

void TrajectoryPlannerPlugin::publish_opened_3d_nodes_map(
    const convert::ROSOccupancyGrid& msg) const
{
    pub_opened_3d_nodes_map_.publish(msg);
}

void TrajectoryPlannerPlugin::publish_search_space_5d_map(
    const convert::ROSOccupancyGrid& msg) const
{
    pub_search_space_5d_map_.publish(msg);
}

void TrajectoryPlannerPlugin::publish_augmented_path(
    const convert::TPAugmentedPath& msg) const
{
    pub_augmented_path_.publish(msg);
}

void TrajectoryPlannerPlugin::publish_plan_5d(
    const convert::ROSPath& msg) const
{
    pub_plan_5d_.publish(msg);
}

void TrajectoryPlannerPlugin::create_publishers(
    const std::string& name)
{
    nh_ = ros::NodeHandle("~/" + name);

    pub_original_occupancy_map_ =
        nh_.advertise<nav_msgs::OccupancyGrid>("original_occupancy_map", 2, true);
    pub_cost_map_ =
        nh_.advertise<nav_msgs::OccupancyGrid>("cost_map", 2, true);
    pub_plan_3d_ =
        nh_.advertise<nav_msgs::Path>("three_dimension_planner_path", 2, true);
    pub_heuristic_3d_ =
        nh_.advertise<nav_msgs::OccupancyGrid>("three_dimension_heuristic", 2, true);
    pub_opened_3d_nodes_map_ =
        nh_.advertise<nav_msgs::OccupancyGrid>("three_dimension_planner_visited_space", 2, true);
    pub_search_space_5d_map_ =
        nh_.advertise<nav_msgs::OccupancyGrid>("five_dimension_planner_search_space", 2, true);
    pub_augmented_path_ =
        nh_.advertise<trajectory_planner_msgs::AugmentedPath>("augmented_path", 10);
    pub_plan_5d_ =
        nh_.advertise<nav_msgs::Path>("five_dimension_planner_path", 2, true);
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    trajectory_planner::TrajectoryPlannerPlugin,
    nav_core::BaseGlobalPlanner)
