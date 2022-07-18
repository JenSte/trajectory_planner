#ifndef TRAJECTORY_PLANNER_PLANNING_3D_HPP
#define TRAJECTORY_PLANNER_PLANNING_3D_HPP

#include "trajectory_planner/costs.hpp"

#include <boost/container/static_vector.hpp>
#include <boost/container_hash/hash.hpp>

#include <tuple>
#include <unordered_map>

// The functions in this name space implement a "three dimensional" search. This
// means that the search happens in on cells (X, Y, Theta), i.e. taking the orientation
// of the robot into account.
namespace trajectory_planner::three
{

// The tree dimensional planner also uses a discretized angle representation,
// but with a much lower granularity (see 'movement_index_count' below).
using MovementIndex = unsigned int;

// The number of different values a variable of the type 'MovementIndex' may
// have (0 .. movement_index_count - 1).
const MovementIndex movement_index_count = 16;

// A pose as used by the three dimensional planner.
struct Pose3D
{
    // The X coordinate of the cell.
    unsigned int x;

    // The Y coordinate of the cell.
    unsigned int y;

    // A value corresponding to the orientation on the cell.
    MovementIndex movement;

    bool operator==(
        const Pose3D& other) const
    {
        return (x == other.x) && (y == other.y) && (movement == other.movement);
    }

    friend bool operator<(
        const Pose3D& lh, const Pose3D& rh)
    {
        return std::tie(lh.x, lh.y, lh.movement) < std::tie(rh.x, rh.y, rh.movement);
    }

    friend std::size_t hash_value(const Pose3D& p)
    {
        std::size_t seed = 0;

        boost::hash_combine(seed, p.x);
        boost::hash_combine(seed, p.y);
        boost::hash_combine(seed, p.movement);

        return seed;
    }
};

// A path planned by the three dimensional planner.
using Path3D = std::vector<Pose3D>;

// The type of a lookup table used to convert between the movement indices
// of the 3D planner and the (more granular) angle indices used by the
// trajectory planner (in particular it's 'Costs' object). This table can
// be indexed by a movement index to get the corresponding angle index.
using AngleIndexLUT = std::array<unsigned int, movement_index_count>;

// The heuristic used by the 3D planner is the 2D distance from the goal.
using HeuristicMap = std::unordered_map<Pose2D, double, boost::hash<Pose2D>>;

// Data type returned from the search in 3D space.
struct SearchResult3D
{
    // The result of the path search, converted back to the data type used by
    // the trajectory planner.
    Path path;

    // Contains the heuristic value for every pose in the path (vector has
    // the exact same length as 'path').
    std::vector<double> heuristic;

    // The cost values of the poses along the path (vector has the exact same
    // length as 'path').
    std::vector<double> cost;

    // The data structure that contains the pre-calculated 2D heuristic values.
    HeuristicMap heuristic_map;

    // Describes the cells the search algorithm looked at during the search. This
    // maps the 2D coordinates to the number of orientations that were checked
    // on that postition. As the maximum number of valid orientations (orientations
    // that do not collide with the map) is 'movement_index_count', the maximum value
    // of a map value is also 'movement_index_count'. Coordinates that were not looked
    // at at all (in no orientation) are not included.
    using OpenedNodesMap = std::unordered_map<Pose2D, unsigned int, boost::hash<Pose2D>>;
    OpenedNodesMap opened_nodes;
};

// Create a lookup table for a given angle granularity.
AngleIndexLUT create_lookup_table(
    unsigned int angle_granularity);

// Create the data structure containting the heuristic values.
HeuristicMap create_heuristic_map(
    const Costs& costs,
    const Pose3D& goal);

// Return the two neighbour candidates for a given pose when moving forward/backward.
std::tuple<Pose3D, Pose3D> linear_neighbours(
    const Pose3D& pose);

// Return the two neighbour candidates for a given pose when turning on the spot.
std::tuple<Pose3D, Pose3D> turn_neighbours(
    const Pose3D& pose);

// Return up to eight neighbouring poses for a given pose that actually exist in the
// costs object.
boost::container::static_vector<Pose3D, 8> neighbours(
    const Costs& costs,
    const AngleIndexLUT& lut,
    const Pose3D& pose);

// Round a pose from the trajectory planner to a pose of the 3D planner, and
// check if it exists in the costs object. Raises an exception if the resulting
// pose is not valid.
Pose3D refine_pose(
    const Costs& costs,
    const AngleIndexLUT& lut,
    const Pose& pose);

// Plan between two poses in three dimensions.
SearchResult3D plan(
    const Costs& costs,
    const Pose& start,
    const Pose& goal);
}

#endif
