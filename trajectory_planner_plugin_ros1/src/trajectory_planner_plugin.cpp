#include <trajectory_planner_plugin/trajectory_planner_plugin.hpp>

namespace trajectory_planner
{

TrajectoryPlannerPlugin::TrajectoryPlannerPlugin()
{
    ROS_WARN("TrajectoryPlannerPlugin::TrajectoryPlannerPlugin()");
}

void TrajectoryPlannerPlugin::initialize(
    std::string name,
    costmap_2d::Costmap2DROS* costmap_ros)
{
}

bool TrajectoryPlannerPlugin::makePlan(
    const geometry_msgs::PoseStamped& start,
    const geometry_msgs::PoseStamped& goal,
    std::vector<geometry_msgs::PoseStamped>& plan)
{
    return false;
}

}

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(
    trajectory_planner::TrajectoryPlannerPlugin,
    nav_core::BaseGlobalPlanner)
