#include "trajectory_planner_plugin/trajectory_planner_plugin.hpp"

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

    logger_ = node_->get_logger().get_child(name_);

    RCLCPP_DEBUG((*logger_), "TrajectoryPlannerPlugin: configure()");

    declare_parameters();
    create_publishers();
}

void TrajectoryPlannerPlugin::cleanup()
{
    RCLCPP_DEBUG((*logger_), "TrajectoryPlannerPlugin: cleanup()");
}

void TrajectoryPlannerPlugin::activate()
{
    RCLCPP_DEBUG((*logger_), "TrajectoryPlannerPlugin: activate()");

    pub_original_occupancy_map_->on_activate();
    pub_cost_map_->on_activate();
    pub_plan_3d_->on_activate();
    pub_heuristic_3d_->on_activate();
    pub_opened_3d_nodes_map_->on_activate();
    pub_search_space_5d_->on_activate();
    pub_augmented_path_->on_activate();
    pub_plan_5d_->on_activate();

    // Read the parameters that are needed now, when creating the planner,
    // and not only later when the actual planning is done.

    int angle_granularity;
    double inscribed_radius;
    double inflation_radius;
    std::string cache_directory;
    std::string debug_directory;
    trajectory_planner::three::CostMap3DType cost_map_3d_type;
    trajectory_planner::three::OrientationCosts3DType orientation_costs_3d_type;

    node_->get_parameter(name_ + ".angle_granularity", angle_granularity);
    node_->get_parameter(name_ + ".inscribed_radius", inscribed_radius);
    node_->get_parameter(name_ + ".inflation_radius", inflation_radius);
    node_->get_parameter(name_ + ".cache_directory", cache_directory);
    node_->get_parameter(name_ + ".debug_directory", debug_directory);

    std::string str;
    node_->get_parameter(name_ + ".cost_map_3d_type", str);
    if (str == "linear") {
        cost_map_3d_type = trajectory_planner::three::CostMap3DType::LINEAR;
    } else if (str == "exponential") {
        cost_map_3d_type = trajectory_planner::three::CostMap3DType::EXPONENTIAL;
    } else {
        cost_map_3d_type = trajectory_planner::three::CostMap3DType::EXPONENTIAL;
        RCLCPP_WARN((*logger_), "Unable to parse cost_map_3d_type, using 'exponental'." );
    }

    node_->get_parameter(name_ + ".orientation_costs_3d_type", str);
    if (str == "identical") {
        orientation_costs_3d_type = trajectory_planner::three::OrientationCosts3DType::IDENTICAL;
    } else if (str == "footprint") {
        orientation_costs_3d_type = trajectory_planner::three::OrientationCosts3DType::FOOTPRINT;
    } else {
        orientation_costs_3d_type = trajectory_planner::three::OrientationCosts3DType::IDENTICAL;
        RCLCPP_WARN((*logger_), "Unable to parse orientation_costs_3d_type, using 'identical'.");
    }

    // (Re-)Start watching the costmap for changes and recreate the planner if needed.
    costmap_observer_.watch_costmap(
        cost_map_3d_type,
        orientation_costs_3d_type,
        angle_granularity,
        inscribed_radius,
        inflation_radius,
        cache_directory,
        debug_directory,
        {},
        costmap_ros_);
}

void TrajectoryPlannerPlugin::deactivate()
{
    RCLCPP_DEBUG((*logger_), "TrajectoryPlannerPlugin: deactivate()");

    pub_original_occupancy_map_->on_deactivate();
    pub_cost_map_->on_deactivate();
    pub_plan_3d_->on_deactivate();
    pub_heuristic_3d_->on_deactivate();
    pub_opened_3d_nodes_map_->on_deactivate();
    pub_search_space_5d_->on_deactivate();
    pub_augmented_path_->on_deactivate();
    pub_plan_5d_->on_deactivate();

    costmap_observer_.stop_watching();
}

nav_msgs::msg::Path TrajectoryPlannerPlugin::createPlan(
    const geometry_msgs::msg::PoseStamped& start_msg,
    const geometry_msgs::msg::PoseStamped& goal_msg)
{
    return create_plan(start_msg, goal_msg);
}

void TrajectoryPlannerPlugin::log_info(
    const std::string& message) const
{
    if (logger_) {
        RCLCPP_INFO_STREAM((*logger_), message);
    }
}

void TrajectoryPlannerPlugin::log_warn(
    const std::string& message) const
{
    if (logger_) {
        RCLCPP_WARN_STREAM((*logger_), message);
    }
}

TrajectoryPlanner::PlanningParameters TrajectoryPlannerPlugin::read_planning_parameters() const
{
    TrajectoryPlanner::PlanningParameters result;

    // Currently not used.
    result.linear_acceleration_maximum = 1.1;
    result.angular_acceleration_maximum = 3.5;

    result.multi_threaded = true;
    node_->get_parameter(name_ + ".split_long_5d_segments", result.split_long_5d_segments);
    node_->get_parameter(name_ + ".extract_3d_opened_nodes", result.extract_3d_opened_nodes);

    node_->get_parameter(name_ + ".inflation_5d_radius", result.inflation_5d_radius);
    node_->get_parameter(name_ + ".inflation_5d_lookahead", result.inflation_5d_lookahead);
    node_->get_parameter(name_ + ".maximum_wheel_velocity", result.maximum_wheel_velocity);
    node_->get_parameter(name_ + ".maximum_wheel_acceleration", result.maximum_wheel_acceleration);
    node_->get_parameter(name_ + ".wheel_distance", result.wheel_distance);

    int linear_velocity_steps;
    node_->get_parameter(name_ + ".linear_velocity_steps", linear_velocity_steps);
    result.linear_velocity_steps = linear_velocity_steps;

    int angular_velocity_steps;
    node_->get_parameter(name_ + ".angular_velocity_steps", angular_velocity_steps);
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

    return result;
}

void TrajectoryPlannerPlugin::publish_original_occupancy_map(
    const nav_msgs::msg::OccupancyGrid& msg) const
{
    if (pub_original_occupancy_map_) {
        pub_original_occupancy_map_->publish(msg);
    }
}

void TrajectoryPlannerPlugin::publish_cost_map(
    const nav_msgs::msg::OccupancyGrid& msg) const
{
    if (pub_cost_map_) {
        pub_cost_map_->publish(msg);
    }
}

void TrajectoryPlannerPlugin::publish_plan_3d(
    const nav_msgs::msg::Path& msg) const
{
    if (pub_plan_3d_) {
        pub_plan_3d_->publish(msg);
    }
}

void TrajectoryPlannerPlugin::publish_heuristic_3d(
    const nav_msgs::msg::OccupancyGrid& msg) const
{
    if (pub_heuristic_3d_) {
        pub_heuristic_3d_->publish(msg);
    }
}

void TrajectoryPlannerPlugin::publish_opened_3d_nodes_map(
    const nav_msgs::msg::OccupancyGrid& msg) const
{
    if (pub_opened_3d_nodes_map_) {
        pub_opened_3d_nodes_map_->publish(msg);
    }
}

void TrajectoryPlannerPlugin::publish_search_space_5d_map(
    const nav_msgs::msg::OccupancyGrid& msg) const
{
    if (pub_search_space_5d_) {
        pub_search_space_5d_->publish(msg);
    }
}

void TrajectoryPlannerPlugin::publish_augmented_path(
    const trajectory_planner_msgs::msg::AugmentedPath& msg) const
{
    if (pub_augmented_path_) {
        pub_augmented_path_->publish(msg);
    }
}

void TrajectoryPlannerPlugin::publish_plan_5d(
    const nav_msgs::msg::Path& msg) const
{
    if (pub_plan_5d_) {
        pub_plan_5d_->publish(msg);
    }
}


void TrajectoryPlannerPlugin::declare_parameters()
{
    // Parameters that influence the creation of the costmap. We are not able to
    // react to changes of them, so we mark them as read-only so that users are
    // not confused when changing them via rqt would not result in any change of
    // the cost map.

    rcl_interfaces::msg::ParameterDescriptor ro_descriptor;
    ro_descriptor.read_only = true;

    ro_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".cost_map_3d_type", rclcpp::ParameterValue("linear"), ro_descriptor);

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".orientation_costs_3d_type", rclcpp::ParameterValue("identical"), ro_descriptor);

    ro_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_INTEGER;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".angle_granularity", rclcpp::ParameterValue(128), ro_descriptor);

    ro_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_DOUBLE;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".inscribed_radius", rclcpp::ParameterValue(0.2), ro_descriptor);

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".inflation_radius", rclcpp::ParameterValue(2.0), ro_descriptor);

    // Unfortunately, rqt ignores the read only value for string parameters, even
    // when the correct type value is specified.
    ro_descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_STRING;

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".cache_directory", rclcpp::ParameterValue(""), ro_descriptor);

    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".debug_directory", rclcpp::ParameterValue(""), ro_descriptor);


    // Parameters that only influence the planning (not the costmap creation). We
    // add ranges and descriptions for each of the parameters.

    rcl_interfaces::msg::ParameterDescriptor descriptor;

    descriptor.type = rcl_interfaces::msg::ParameterType::PARAMETER_BOOL;
    descriptor.description = "Whether or not to slit and process long 5D segments separately";
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".split_long_5d_segments", rclcpp::ParameterValue(false), descriptor);

    descriptor.description = "Whether or not to publish a map showing all cells looked at by the 3D planner";
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".extract_3d_opened_nodes", rclcpp::ParameterValue(false), descriptor);

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

    descriptor.description = "The maximum velocity of a wheel of the vehicle, in meter/second";
    descriptor.floating_point_range.at(0).from_value = 0.02;
    descriptor.floating_point_range.at(0).to_value = 2.0;
    descriptor.floating_point_range.at(0).step = 0.02;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".maximum_wheel_velocity", rclcpp::ParameterValue(0.5), descriptor);

    descriptor.description = "The maximum acceleration of a wheel of the vehicle, in meter/(second^2)";
    descriptor.floating_point_range.at(0).from_value = 0.02;
    descriptor.floating_point_range.at(0).to_value = 2.0;
    descriptor.floating_point_range.at(0).step = 0.02;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".maximum_wheel_acceleration", rclcpp::ParameterValue(0.5), descriptor);

    descriptor.description = "The distance between the two driving wheels of the vehicle, in meter";
    descriptor.floating_point_range.at(0).from_value = 0.1;
    descriptor.floating_point_range.at(0).to_value = 2.0;
    descriptor.floating_point_range.at(0).step = 0.01;
    nav2_util::declare_parameter_if_not_declared(
        node_, name_ + ".wheel_distance", rclcpp::ParameterValue(0.4), descriptor);

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

void TrajectoryPlannerPlugin::create_publishers()
{
    pub_original_occupancy_map_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name_ + "/original_occupancy_map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_cost_map_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name_ + "/cost_map",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_plan_3d_ =
        node_->create_publisher<nav_msgs::msg::Path>(
            "~/" + name_ + "/three_dimension_planner_path",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_heuristic_3d_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name_ + "/three_dimension_heuristic",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_opened_3d_nodes_map_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name_ + "/three_dimension_planner_visited_space",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_search_space_5d_ =
        node_->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "~/" + name_ + "/five_dimension_planner_search_space",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());

    pub_augmented_path_ =
        node_->create_publisher<trajectory_planner_msgs::msg::AugmentedPath>(
            "~/" + name_ + "/augmented_path",
            10);

    pub_plan_5d_ =
        node_->create_publisher<nav_msgs::msg::Path>(
            "~/" + name_ + "/five_dimension_planner_path",
            rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable());
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
    trajectory_planner::TrajectoryPlannerPlugin,
    nav2_core::GlobalPlanner)
