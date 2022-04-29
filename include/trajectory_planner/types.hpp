#ifndef TRAJECTORY_PLANNER_TYPES_HPP
#define TRAJECTORY_PLANNER_TYPES_HPP

#include <boost/container_hash/hash.hpp>

#include <tuple>
#include <vector>

namespace trajectory_planner
{

// A point, consisting of a X and Y coordinate, in meter.
using Point = std::tuple<double, double>;

// A (closed, first and last point are considered to be connected) polygon.
using Polygon = std::vector<Point>;

// The pose of a robot.
struct Pose
{
    // The X coordinate of the cell.
    unsigned int x;

    // The Y coordinate of the cell.
    unsigned int y;

    // The index of the angle, multiples of the fraction of the whole circle.
    unsigned int angle_index;

    friend bool operator==(
        const Pose& lh, const Pose& rh)
    {
        return std::tie(lh.x, lh.y, lh.angle_index) == std::tie(rh.x, rh.y, rh.angle_index);
    }
};

// Data structure to describe a single cell of the map,
// not taking the orientation into account.
struct Pose2D
{
    // The X coordinate of the cell.
    unsigned int x;

    // The Y coordinate of the cell.
    unsigned int y;

    bool operator==(
        const Pose2D& other) const
    {
        return (x == other.x) && (y == other.y);
    }

    friend bool operator<(
        const Pose2D& lh, const Pose2D& rh)
    {
        return std::tie(lh.x, lh.y) < std::tie(rh.x, rh.y);
    }

    friend std::size_t hash_value(const Pose2D& p)
    {
        std::size_t seed = 0;

        boost::hash_combine(seed, p.x);
        boost::hash_combine(seed, p.y);

        return seed;
    }
};

// A path from one pose to another. The first element is the start pose, the
// last element is the goal.
using Path = std::vector<Pose>;

}

#endif
