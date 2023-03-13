#ifndef TRAJECTORY_PLANNER_COSTS_HPP
#define TRAJECTORY_PLANNER_COSTS_HPP

#include "trajectory_planner/types.hpp"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>

#include <cstdint>
#include <unordered_map>
#include <unordered_set>

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

    // Initialize a cost object. 'angle_granularity' is the number of steps the whole
    // circle (2 * PI) is divided in. 'goal_turn_penalty_distance' is the distance to
    // the goal (in cells) for which a penalty to turn shall be applied.
    Costs(
        unsigned int angle_granularity,
        unsigned int goal_turn_penalty_distance);

    // Store the cost value of a pose.
    void set_3d_cost(
        unsigned int x,
        unsigned int y,
        unsigned int angle_index,
        double cost);

    // Return the cost value of a pose.
    //
    // If the cost of the given pose has never been set, 'invalid_cost_3d' is returned.
    double get_3d_cost(
        unsigned int x,
        unsigned int y,
        unsigned int angle_index) const;

    // Convenience overload.
    double get_3d_cost(
        const Pose& pose) const;

    // Return the number of steps the whole circle is divided in.
    unsigned int angle_granularity() const;

    // Return a distance in cells to the goal to apply a turn penaltiy.
    unsigned int goal_turn_penalty_distance() const;

    // Create an intersection between the costs in this object and the
    // coordinates passed in.
    Costs intersect(
        std::vector<Pose2D> poses) const;

    // Inserts all 2D poses that are contained in the cost structure into the given set.
    void export_poses(
        std::unordered_set<Pose2D, boost::hash<Pose2D>>& poses_set) const;

    // Value used for positions that can not be occupied.
    static const double invalid_cost_3d;

private:

    // The data type to store the cost values internally.
    using CostType3D = uint16_t;

    // Data type to hold the costs for a location on the map. The length of this
    // vector is 'angle_granularity_', for orientations that can not be occupied
    // the value is set to 'internal_invalid_cost_3d'.
    using CostVector3D = std::vector<CostType3D>;

    // Data type to hold the X and Y coordinates.
    using Location = std::pair<unsigned int, unsigned int>;

    // The value for marking an invalid orientation in a 'CostVector'.
    static const CostType3D internal_invalid_cost_3d;

    // Number of steps to divide the whole circle with.
    unsigned int angle_granularity_;

    // Cell distance to the goal to apply a penalty on pure turns.
    unsigned int goal_turn_penalty_distance_;

    // Stores the cost values.
    std::unordered_map<Location, CostVector3D, boost::hash<Location>> costs_3d_;

    // Convert the internal cost type to a double value.
    double internal_cost_3d_to_double_cost(
        CostType3D c) const;

    // Convert a double value to the internal cost type.
    CostType3D double_cost_to_internal_cost_3d(
        double c) const;

    // Serialization support.
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/)
    {
        ar & angle_granularity_;
        ar & goal_turn_penalty_distance_;
        ar & costs_3d_;
    }
};

}

#endif
