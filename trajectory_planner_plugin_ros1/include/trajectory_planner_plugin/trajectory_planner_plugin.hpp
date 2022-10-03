#ifndef TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP
#define TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP

#include "trajectory_planner_plugin/plugin.hpp"

#include <nav_core/base_global_planner.h>

namespace trajectory_planner
{

// move_it (ROS1) plugin for the trajectory planner.
class TrajectoryPlannerPlugin: public nav_core::BaseGlobalPlanner, public Plugin
{
public:
    TrajectoryPlannerPlugin();

    void initialize(
        std::string name,
        costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(
        const geometry_msgs::PoseStamped& start_msg,
        const geometry_msgs::PoseStamped& goal_msg,
        std::vector<geometry_msgs::PoseStamped>& plan);

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
        const nav_msgs::OccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_cost_map(
        const nav_msgs::OccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_plan_3d(
        const convert::ROSPath& msg) const override;

    // Implementation of the base class function.
    void publish_heuristic_3d(
        const convert::ROSOccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_opened_3d_nodes_map(
        const convert::ROSOccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_search_space_5d_map(
        const convert::ROSOccupancyGrid& msg) const override;

    // Implementation of the base class function.
    void publish_augmented_path(
        const convert::TPAugmentedPath& msg) const override;

    // Implementation of the base class function.
    void publish_plan_5d(
        const convert::ROSPath& msg) const override;

private:

    // Create all the (debug) publishers that are used in this class.
    void create_publishers(
        const std::string& name);

    // The (private) node handle used in this plugin.
    ros::NodeHandle nh_;

    // Publisher for the origin occupancy map, published for debug purposes.
    ros::Publisher pub_original_occupancy_map_;

    // Publisher for the cost map, published for debug purposes.
    ros::Publisher pub_cost_map_;

    // Publisher for the plan found by the 3D planner, published for debug purposes.
    ros::Publisher pub_plan_3d_;

    // Publisher for the (2D) heuristic used by the 3D planner, published for debug purposes.
    ros::Publisher pub_heuristic_3d_;

    // Publisher for a map showing the space visited by the 3-dimensional planner,
    // published for debug purposes.
    ros::Publisher pub_opened_3d_nodes_map_;

    // Publisher for the area inflated around the 3D path that is searched by the 5D
    // planner, published for debug purposes.
    ros::Publisher pub_search_space_5d_map_;

    // Publisher for the result of the search with additional information
    // to debug the planner.
    ros::Publisher pub_augmented_path_;

    // Publisher for the plan found by the 5D planner, published for debug purposes.
    ros::Publisher pub_plan_5d_;
};

}

#endif
