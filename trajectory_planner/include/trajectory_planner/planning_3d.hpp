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

// The angles in radians that correspond to a 3D movement index.
const double angle_lut[movement_index_count] = {
    // 1st quadrant.
    atan2(0, 1),
    atan2(1, 2),
    atan2(1, 1),
    atan2(2, 1),
    // 2nd quadrant.
    atan2(1, 0),
    atan2(2, -1),
    atan2(1, -1),
    atan2(1, -2),
    // 3rd quadrant.
    atan2(0, -1),
    atan2(-1, -2),
    atan2(-1, -1),
    atan2(-2, -1),
    // 4th quadrant.
    atan2(-1, 0),
    atan2(-2, 1),
    atan2(-1, 1),
    atan2(-1, 2)
};

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

// Holds the heuristic values used by the 3D planner.
class Heuristic
{
public:

    // Create a new Heuristic instance.
    Heuristic(
        const Costs& costs,
        const Pose3D& goal);

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

// Data type returned from the search in 3D space.
struct SearchResult3D
{
    SearchResult3D(
        Heuristic heuristic)
        : heuristic(std::move(heuristic))
    {
    }

    // The result of the path search, converted back to the data type used by
    // the trajectory planner.
    Path path;

    // Contains the heuristic value for every pose in the path (vector has
    // the exact same length as 'path').
    std::vector<double> path_heuristic;

    // The cost values of the poses along the path (vector has the exact same
    // length as 'path').
    std::vector<double> cost;

    // The data structure that contains the pre-calculated 2D heuristic values.
    Heuristic heuristic;

    // Describes the cells the search algorithm looked at during the search. This
    // maps the 2D coordinates to the number of orientations that were checked
    // on that postition. As the maximum number of valid orientations (orientations
    // that do not collide with the map) is 'movement_index_count', the maximum value
    // of a map value is also 'movement_index_count'. Coordinates that were not looked
    // at at all (in no orientation) are not included.
    using OpenedNodesMap = std::unordered_map<Pose2D, unsigned int, boost::hash<Pose2D>>;
    OpenedNodesMap opened_nodes;
};

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
    const Pose3D& pose);

// Round a pose from the trajectory planner to a pose of the 3D planner, and
// check if it exists in the costs object. Raises an exception if the resulting
// pose is not valid.
Pose3D refine_pose(
    const Costs& costs,
    const Pose& pose);

// Plan between two poses in three dimensions.
SearchResult3D plan(
    const Costs& costs,
    const Pose& start,
    const Pose& goal,
    bool extract_opened_nodes);
}

#endif
