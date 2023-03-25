#ifndef TRAJECTORY_PLANNER_COSTS_HPP
#define TRAJECTORY_PLANNER_COSTS_HPP

#include "trajectory_planner/types.hpp"

#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/serialization/vector.hpp>

#include <cstdint>
#include <limits>
#include <unordered_set>

namespace trajectory_planner
{

// A class to hold cost values for a three dimensional pose (x, y, theta).
//
// The X and Y coordiantes are positive integer values, and the angle is
// represented as a positive integer value that denotes the number of
// fractions of the whole circle. For the values of theta (strictly speaking, the
// integer values corresponding to these angles) an integer value is stored holding
// an actual cost value. For the angles values ('0' to 'angle_granularity - 1') that
// are used by the 5D planner only a single bit that describes if the robot can be
// placed at this pose.
class Costs
{
public:

    // The data type to store the cost values used by the 3D planner.
    //
    // Values for variables of this type are in the range from '0' to
    // 'invalid_3d_cost - 1', 'invalid_3d_cost' is used to mark invalid poses.
    using CostType3D = uint8_t;

    // Value used to denote that the robot can not be placed on a given coordinate.
    static constexpr CostType3D invalid_3d_cost = std::numeric_limits<CostType3D>::max();

    // The data type used to hold the costs for a single orientation used by the
    // 3D planner.
    using CostVector3D = std::vector<CostType3D>;

    // The data type used to hold the occupancy data used by the 5D planner.
    //
    // These vectors have the size 'floorplan_width * floorplan_height' and each
    // element holds the information if the robot can be placed ('true') at a
    // given X/Y coordinate or not.
    using CostVector5D = std::vector<bool>;

    // Initialize a cost object. 'angle_granularity' is the number of steps the whole
    // circle (2 * PI) is divided in. 'goal_turn_penalty_distance' is the distance to
    // the goal (in cells) for which a penalty to turn shall be applied.
    Costs(
        size_t floorplan_width,
        size_t floorplan_height,
        unsigned int angle_granularity,
        unsigned int goal_turn_penalty_distance);

    // Create a 3D cost vector, correctly sized and initialized to the invalid cost value.
    CostVector3D create_3d_cost_vector() const;

    // Set the 3D cost vector for a given orientation.
    void set_3d_cost_vector(
        unsigned int movement_index,
        CostVector3D cost_vector);

    // Return the cost value of a pose.
    //
    // If the cost of the given pose has never been set, 'invalid_cost_3d' is returned.
    CostType3D get_3d_cost(
        unsigned int x,
        unsigned int y,
        unsigned int angle_index) const;

    // Store the value at a given position in the given cost vector.
    void set_3d_cost(
        CostVector3D& cost_vector,
        unsigned int x,
        unsigned int y,
        float cost) const;

    // Convert a value of the type used to store the 3D costs to a float.
    static float cost_3d_to_float(
        const CostType3D c)
    {
        return static_cast<float>(c) / static_cast<float>(invalid_3d_cost - 1);
    }

    // Create a 5D cost vector, correctly sized and initialized to 'false'.
    CostVector5D create_5d_cost_vector() const;

    // Set the 5D cost vector for a given angle index.
    void set_5d_cost_vector(
        unsigned int angle_index,
        CostVector5D cost_vector);

    // Get the information if the robot can be placed on a given pose.
    //
    // If the result of this function is 'true', the robot can be placed on
    // the pose, otherwise not.
    bool get_5d_cost(
        unsigned int x,
        unsigned int y,
        unsigned int angle_index) const;

    // Mark the value at a given position as being able to hold the robot.
    void set_5d_cost(
        CostVector5D& cost_vector,
        unsigned int x,
        unsigned int y) const;

    // Return the number of steps the whole circle is divided in.
    unsigned int angle_granularity() const;

    // Return a distance in cells to the goal to apply a turn penaltiy.
    unsigned int goal_turn_penalty_distance() const;

    // Intersect the 5D costs stored in this object with the passed coordinates. All
    // costs (occupancy) values in the result will be set to "occupied" if the pose
    // was not in the input argument. The 3D costs in the returned object are all empty.
    Costs intersect_5d_costs(
        std::vector<Pose2D> poses) const;

    // Inserts all coordinates of the 5D poses that are contained in the cost structure
    // into the given set.
    void export_5d_poses(
        std::unordered_set<Pose2D, boost::hash<Pose2D>>& poses_set) const;

    // Return the width of the floorplan.
    size_t width() const
    {
        return floorplan_width_;
    }

    // Return the height of the floorplan.
    size_t height() const
    {
        return floorplan_height_;
    }

private:

    // Number of steps to divide the whole circle with.
    unsigned int angle_granularity_;

    // The width of the floorplan this object holds the costs for.
    size_t floorplan_width_;

    // The height of the floorplan this object holds the costs for.
    size_t floorplan_height_;

    // Cell distance to the goal to apply a penalty on pure turns.
    unsigned int goal_turn_penalty_distance_;

    // Stores the cost values for the 3D planner.
    //
    // Maps an movement index as used by the 3D planner to a vector containing
    // the costs of placing the robot on a given coordinate.
    std::vector<CostVector3D> costs_3d_;

    // Stores the cost (occupancy) values for the 5D planner.
    //
    // Maps angle indices for the orientation of the robot to a 5D cost vector,
    // that holds the occupancy data for the given orientation.
    std::vector<CostVector5D> costs_5d_;

    // Serialization support.
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive& ar, const unsigned int /*version*/)
    {
        ar & angle_granularity_;
        ar & goal_turn_penalty_distance_;
        ar & costs_3d_;
        ar & costs_5d_;
    }
};

}

#endif
