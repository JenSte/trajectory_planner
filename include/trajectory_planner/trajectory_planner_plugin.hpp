#ifndef TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP
#define TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP

#include "trajectory_planner/msg/augmented_path.hpp"
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
        const geometry_msgs::msg::PoseStamped& start_msg,
        const geometry_msgs::msg::PoseStamped& goal_msg) override;

private:

    // Pointer to the node handle passed to the 'configure()' function.
    nav2_util::LifecycleNode::SharedPtr node_;

    // Pointer to the costmap_ros passed to the 'configure()' function.
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;

    // Logger object used in the whole plugin.
    std::optional<rclcpp::Logger> logger_;

    // The number of steps to divide the whole circle.
    int angle_granularity_;

    // The distance for up to which obstacles are inflated when calculating the
    // cost map.
    double inflation_radius_;

    // The name of a directory to cache planners.
    std::string cache_directory_;

    // The name of a directory to dump debug information to.
    std::string debug_directory_;

    // A pointer to the planner. Protected by 'planner_mutex_'.
    std::unique_ptr<TrajectoryPlanner> planner_;

    // Mutex to protect 'planner_', that is also accessed by the background thread.
    std::mutex planner_mutex_;

    // Publisher for the origin occupancy map, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_original_occupancy_map_;

    // Publisher for the cost map, published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_cost_map_;

    // Publishes the created plan for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr
        pub_path_;

    // Publish a map showing the space visited by the 3-dimensional planner,
    // published for debug purposes.
    rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::OccupancyGrid>::SharedPtr
        pub_3d_debug_map_;

    // Publish the result of the search with additional information
    // to debug the planner.
    rclcpp_lifecycle::LifecyclePublisher<trajectory_planner::msg::AugmentedPath>::SharedPtr
        pub_augmented_path_;

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

    // Convert a ROS pose message to the pose structure used by the planner.
    Pose convert_pose_msg(
        const nav2_costmap_2d::Costmap2D* costmap,
        const geometry_msgs::msg::PoseStamped& pose_msg,
        unsigned int angle_granularity) const;

    // Convert a pose structure as used by the planner to a ROS pose message.
    geometry_msgs::msg::PoseStamped convert_pose(
        const nav2_costmap_2d::Costmap2D* costmap,
        const std::string& frame_id,
        const Pose& pose,
        unsigned int angle_granularity) const;

    // Create a buffer that contains the value "1.0" where there are objects
    // in the given costmap.
    Buffer<double> create_occupancy_map(
        const nav2_costmap_2d::Costmap2D* costmap) const;

    // Convert a buffer object back to an occupancy grid ROS message.
    nav_msgs::msg::OccupancyGrid convert_buffer(
        const Buffer<double>& buffer,
        const std::string& frame_id,
        double resolution,
        double origin_x,
        double origin_y) const;

    // Convert the map containing the opened nodes to an occupancy grid ROS message.
    nav_msgs::msg::OccupancyGrid convert_opened_nodes(
        const std::map<three::Pose2D, unsigned int>& opened_nodes,
        size_t width,
        size_t height,
        const std::string& frame_id,
        double resolution,
        double origin_x,
        double origin_y) const;

    // Create an augmented path message.
    msg::AugmentedPath create_augmented_path_message(
        const nav2_costmap_2d::Costmap2D* costmap,
        const std::string& frame_id,
        unsigned int angle_granularity,
        const three::SearchResult3D& search_result) const;

    // Create a hash used to identify the costmap.
    std::string hash_costmap(
        const Polygon& footprint,
        nav2_costmap_2d::Costmap2D* costmap) const;

    // Create a plan using the given planner.
    nav_msgs::msg::Path plan(
        nav2_costmap_2d::Costmap2D* costmap,
        const std::string& costmap_frame_id,
        TrajectoryPlanner* planner,
        const geometry_msgs::msg::PoseStamped& start_msg,
        const geometry_msgs::msg::PoseStamped& goal_msg) const;

    // Function that implements the planner update.
    void planner_update_thread_function();

    // Return the file name that shall be used to cache a planner's cost object.
    std::string cache_file_name(
        const std::string& hash) const;
};

}

#endif
