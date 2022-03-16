#ifndef TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP
#define TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP

#include "trajectory_planner/trajectory_planner.hpp"

#include "nav2_core/global_planner.hpp"

#include <optional>
#include <thread>

namespace trajectory_planner
{

// Navigation stack plugin for the trajectory planner.
class TrajectoryPlannerPlugin: public nav2_core::GlobalPlanner
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
        const geometry_msgs::msg::PoseStamped& start,
        const geometry_msgs::msg::PoseStamped& goal) override;

private:

    // Pointer to the node handle passed to the 'configure()' function.
    nav2_util::LifecycleNode::SharedPtr node_;

    // Pointer to the costmap_ros passed to the 'configure()' function.
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    // Logger object used in the whole plugin.
    std::optional<rclcpp::Logger> logger_;

    // The number of steps to divide the whole circle.
    int angle_granularity_;

    // The name of a directroy to dump debug information to.
    std::string debug_directory_;

    // A pointer to the planner. Protected by 'planner_mutex_'.
    std::unique_ptr<TrajectoryPlanner> planner_;

    // Mutex to protect 'planner_', that is also accessed by the background thread.
    std::mutex planner_mutex_;

    // Publisher for the origin occupancy map, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_original_occupancy_map_;

    // Publisher for the origin cost map, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_original_cost_map_;

    // Background thread that periodically checks for changes of the costmap
    // and updates the planner if necessary.
    std::thread planner_update_thread_;

    // Mutex to be used together with 'planner_update_thread_notification_'.
    std::mutex planner_update_thread_mutex_;

    // Condition variable to communicate with the planner update thread.
    std::condition_variable planner_update_thread_notification_;

    // Set to 'true' to notify the background thread to stop.
    bool planner_update_thread_quit_ = false;

    // Convert a polygon message to the planer's internal representation.
    Polygon convert_polygon_msg(const geometry_msgs::msg::Polygon& polygon) const;

    // Create a buffer that contains the value "1.0" where there are objects
    // in the given costmap.
    Buffer<double> create_occupancy_map(
        const nav2_costmap_2d::Costmap2D* costmap) const;

    // Create a buffer that contains values from "0.0" (low cost) to "1.0" (high cost)
    // from the given costmap.
    Buffer<double> create_cost_map(
        const nav2_costmap_2d::Costmap2D* costmap) const;

    // Convert a buffer object back to an occupancy grid ROS message.
    nav_msgs::msg::OccupancyGrid convert_buffer(
        const Buffer<double>& buffer,
        const std::string& frame_id,
        double resolution,
        double origin_x,
        double origin_y) const;

    // Create a hash used to identify the costmap.
    std::string hash_costmap(
        nav2_costmap_2d::Costmap2D* costmap) const;

    // Function that implements the planner update.
    void planner_update_thread_function();
};

}

#endif
