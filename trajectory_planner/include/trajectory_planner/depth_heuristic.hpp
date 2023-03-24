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
    DepthHeuristic(
        const Costs& costs,
        const Pose2D& goal);

    // Return the heuristic value of a given pose.
    double get_value(
        unsigned int x,
        unsigned int y) const;

    // Return the biggest contained heuristic value.
    double maximum_value() const
    {
        return maximum_value_;
    }

private:

    // Set the heuristic value of a coordinate.
    void set_value(
        unsigned int x,
        unsigned int y,
        double value);

    // The width of the cost object this object was created from.
    size_t width_;

    // The height of the cost object this object was created from.
    size_t height_;

    // The heuristic used by the 3D planner is the 2D distance from the goal.
    std::vector<double> values_;

    // The maximum distance in 'values_'.
    double maximum_value_;
};

}

#endif
