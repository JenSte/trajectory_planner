#ifndef TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP
#define TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_PLUGIN_HPP

#include "trajectory_planner/trajectory_planner.hpp"

#include <nav_core/base_global_planner.h>

#include <optional>
#include <thread>

namespace trajectory_planner
{

// move_it (ROS1) plugin for the trajectory planner.
class TrajectoryPlannerPlugin: public nav_core::BaseGlobalPlanner
{
public:
    TrajectoryPlannerPlugin();

    void initialize(
        std::string name,
        costmap_2d::Costmap2DROS* costmap_ros);

    bool makePlan(
        const geometry_msgs::PoseStamped& start,
        const geometry_msgs::PoseStamped& goal,
        std::vector<geometry_msgs::PoseStamped>& plan);
};

}

#endif
