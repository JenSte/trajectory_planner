#ifndef TRAJECTORY_PLANNER_PLUGIN_HPP
#define TRAJECTORY_PLANNER_PLUGIN_HPP

#include "trajectory_planner_plugin/costmap_observer.hpp"

namespace trajectory_planner
{

// A generic plugin that contains code for both ROS1 and ROS2.
class Plugin
{
public:

    Plugin();

    virtual ~Plugin();

    CostmapObserver costmap_observer_;

protected:

    // Function for the ROS version-specific plugin to create a plan.
    convert::ROSPath create_plan(
        const convert::ROSPoseStamped& start_msg,
        const convert::ROSPoseStamped& goal_msg) const;

    virtual void log_info(
        const std::string& message) const = 0;

    virtual void log_warn(
        const std::string& message) const = 0;

    virtual void publish_original_occupancy_map(
        const convert::ROSOccupancyGrid& msg) const = 0;

    virtual void publish_cost_map(
        const convert::ROSOccupancyGrid& msg) const = 0;

    virtual void publish_plan_3d(
        const convert::ROSPath& msg) const = 0;

    virtual void publish_heuristic_3d(
        const convert::ROSOccupancyGrid& msg) const = 0;

    virtual void publish_opened_3d_nodes_map(
        const convert::ROSOccupancyGrid& msg) const = 0;

    virtual void publish_search_space_5d_map(
        const convert::ROSOccupancyGrid& msg) const = 0;

    virtual void publish_augmented_path(
        const convert::TPAugmentedPath& msg) const = 0;

    virtual void publish_plan_5d(
        const convert::ROSPath& msg) const = 0;

    virtual TrajectoryPlanner::PlanningParameters read_planning_parameters() const = 0;

    convert::CostmapROSPointer costmap_ros_ = nullptr;

private:

    // Helper function for 'create_plan' to actually call the planner.
    convert::ROSPath plan(
        const convert::Costmap2D* costmap_2d,
        const TrajectoryPlanner* planner,
        const convert::ROSPoseStamped& start_msg,
        const convert::ROSPoseStamped& goal_msg,
        const TrajectoryPlanner::PlanningParameters& planning_parameters) const;

    // Makes sure that the given parameters have sane values.
    void cleanup_planning_parameters(
        TrajectoryPlanner::PlanningParameters& parameters) const;

    // Log the planning parameters.
    void log_planning_parameters(
        const TrajectoryPlanner::PlanningParameters& parameters) const;

    // Publish the augmented path messages for a planning result.
    void publish_augmented_path_messages(
        const convert::Costmap2D* costmap,
        const std::string& frame_id,
        unsigned int angle_granularity,
        const Pose& start,
        const Pose& goal,
        const TrajectoryPlanner::Result& result) const;
};

}

#endif
