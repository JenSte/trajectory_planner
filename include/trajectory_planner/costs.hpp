#ifndef TRAJECTORY_PLANNER_COSTS_HPP
#define TRAJECTORY_PLANNER_COSTS_HPP

#include "trajectory_planner/types.hpp"

#include <map>

namespace trajectory_planner
{

// A class to hold cost values for a three dimensional pose (x, y, theta).
//
// The X and Y coordiantes are positive integer values, and the angle is
// represented as a positive integer value that denotes the number of
// fractions of the whole circle.
class Costs
{
public:

    // Initialize a cost object, 'angle_granularity' is the number of steps
    // the whole circle (2 * PI) is divided.
    Costs(
        unsigned int angle_granularity);

    // Store the cost value of a pose.
    void set_cost(
        unsigned int x,
        unsigned int y,
        unsigned int angle_index,
        double cost);

    // Return the cost value of a pose.
    //
    // If the cost of the given pose has never been set, 'invalid_cost' is returned.
    double get_cost(
        unsigned int x,
        unsigned int y,
        unsigned int angle_index) const;

    // Convenience overload.
    double get_cost(
        const Pose& pose) const;

    // Return the number of steps the whole circle is divided in.
    unsigned int angle_granularity() const;

    // Value used for positions that can not be occupied.
    static const double invalid_cost;

private:

    // Data type to hold the costs for a location on the map. The length of this
    // vector is 'angle_granularity_', for orientations that can not be occupied
    // the value is set to 'invalid_cost'.
    using CostVector = std::vector<double>;

    // Data type to hold the X and Y coordinates.
    using Location = std::tuple<unsigned int, unsigned int>;

    // Number of steps to divide the whole circle with.
    unsigned int angle_granularity_;

    // Stores the cost values.
    std::map<Location, CostVector> costs_;
};

}

#endif
