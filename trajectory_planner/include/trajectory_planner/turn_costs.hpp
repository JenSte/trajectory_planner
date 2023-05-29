#ifndef TRAJECTORY_PLANNER_TURN_COSTS_HPP
#define TRAJECTORY_PLANNER_TURN_COSTS_HPP

#include "trajectory_planner/depth_heuristic.hpp"

namespace trajectory_planner
{

// Simple class to hold the additional costs that are added
// by the 3D planner to move turns away from the goal pose.
class TurnCosts
{
public:

    // Create a new TurnCosts instance.
    //
    // Takes the distance values in the DepthHeuristic and
    // creates a simple cost function derived said distances,
    // that are the highest around the goal, and fall off
    // towards the start pose.
    TurnCosts(
        const DepthHeuristic& heuristic,
        const float goal_distance);

    // Return the turn costs value of a given pose.
    float get_value(
        unsigned int x,
        unsigned int y) const;

    // Return the width of the floorplan this heuristic was created for.
    size_t width() const;

    // Return the height of the floorplan this heuristic was created for.
    size_t height() const;

private:

    // Set the turn cost value of a coordinate.
    void set_value(
        unsigned int x,
        unsigned int y,
        float value);

    // The width of the cost object this object was created from.
    size_t width_;

    // The height of the cost object this object was created from.
    size_t height_;

    // Two-dimensional vector to hold the turn costs.
    std::vector<float> turn_costs_;
};

}

#endif
