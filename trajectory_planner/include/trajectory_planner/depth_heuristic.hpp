#ifndef TRAJECTORY_PLANNER_DEPTH_HEURISTIC_HPP
#define TRAJECTORY_PLANNER_DEPTH_HEURISTIC_HPP

#include "trajectory_planner/costs.hpp"

namespace trajectory_planner
{

// A class to hold heuristic values for the search, derived
// by calculating the distance (depth) from a given goal.
class DepthHeuristic
{
public:

    // Create a new DepthHeuristic instance.
    //
    // Takes the valid poses from 'costs', starting at 'goal', and
    // then calculates the distance to the goal pose for all reachable
    // poses. 'costs_3d' determines if the 3D or 5D costs from the
    // 'costs' object is used to determine if a pose is valid.
    DepthHeuristic(
        const Costs& costs,
        const Pose2D& goal,
        bool costs_3d);

    // Return the heuristic value of a given pose.
    float get_value(
        unsigned int x,
        unsigned int y) const;

    // Return the biggest contained heuristic value.
    float maximum_value() const
    {
        return maximum_value_;
    }

private:

    // Set the heuristic value of a coordinate.
    void set_value(
        unsigned int x,
        unsigned int y,
        float value);

    // Write a debug image that shows the current state of the heuristic. Used
    // to debug the algorithm used to create the heuristic.
    void write_debug_image(
        size_t iteration,
        const std::unordered_set<Pose2D, boost::hash<Pose2D>>& frontier,
        const std::unordered_set<Pose2D, boost::hash<Pose2D>>& old_frontier) const;

    // The width of the cost object this object was created from.
    size_t width_;

    // The height of the cost object this object was created from.
    size_t height_;

    // The heuristic used by the 3D planner is the 2D distance from the goal.
    std::vector<float> values_;

    // The maximum distance in 'values_'.
    float maximum_value_;
};

}

#endif
