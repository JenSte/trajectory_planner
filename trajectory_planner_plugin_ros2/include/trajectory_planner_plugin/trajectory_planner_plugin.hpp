#ifndef TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP
#define TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP

#include "trajectory_planner_plugin/plugin.hpp"

#include "nav2_core/global_planner.hpp"

#include <optional>

namespace trajectory_planner
{

// nav2 (ROS2) plugin for the trajectory planner.
class TrajectoryPlannerPlugin: public nav2_core::GlobalPlanner, public Plugin
{
public:

    void configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr& parent,
        std::string name,
        std::shared_ptr<tf2_ros::Buffer> tf,
        std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

    void cleanup() override;

    void activate() override;

    void deactivate() override;

    nav_msgs::msg::Path createPlan(
        const geometry_msgs::msg::PoseStamped& start_msg,
        const geometry_msgs::msg::PoseStamped& goal_msg) override;

protected:

    // Implementation of the base class function.
    void log_info(
        const std::string& message) const override;

    // Implementation of the base class function.
    void log_warn(
        const std::string& message) const override;

    // Implementation of the base class function.
    TrajectoryPlanner::PlanningParameters read_planning_parameters() const override;

    // Implementation of the base class function.
    void publish_original_occupancy_map(
        const nav_msgs::msg::OccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_cost_map(
        const nav_msgs::msg::OccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_plan_3d(
        const nav_msgs::msg::Path& msg) const override;

    // Implementation of the base class function.
    void publish_heuristic_3d(
        const nav_msgs::msg::OccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_opened_3d_nodes_map(
        const nav_msgs::msg::OccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_search_space_5d_map(
        const nav_msgs::msg::OccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_augmented_path(
        const trajectory_planner_msgs::msg::AugmentedPath& msg) const override;

    // Implementation of the base class function.
    void publish_plan_5d(
        const nav_msgs::msg::Path& msg) const override;

private:

    // Declare all the used ROS parameters and also read the ones that influence
    // the behaviour of the costmap creation.
    void declare_parameters();

    // Create all the (debug) publishers that are used in this class.
    void create_publishers();

    // Pointer to the node handle passed to the 'configure()' function.
    nav2_util::LifecycleNode::SharedPtr node_;

    // The name of the node.
    std::string name_;

    // Logger object used in the whole plugin.
    std::optional<rclcpp::Logger> logger_;

    // Publisher for the origin occupancy map, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_original_occupancy_map_;

    // Publisher for the cost map, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_cost_map_;

    // Publisher for the plan found by the 3D planner, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
        pub_plan_3d_;

    // Publisher for the (2D) heuristic used by the 3D planner, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_heuristic_3d_;

    // Publisher for a map showing the space visited by the 3-dimensional planner,
    // published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_opened_3d_nodes_map_;

    // Publisher for the area inflated around the 3D path that is searched by the 5D
    // planner, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_search_space_5d_;

    // Publisher for the result of the search with additional information
    // to debug the planner.
    rclcpp_lifecycle::LifecyclePublisher<trajectory_planner_msgs::msg::AugmentedPath>::SharedPtr
        pub_augmented_path_;

    // Publisher for the plan found by the 5D planner, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
        pub_plan_5d_;
};

}

#endif
