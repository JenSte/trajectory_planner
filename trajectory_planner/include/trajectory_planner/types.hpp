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

// A pose as used by the five dimensional planner.
struct Pose5D
{
    // The X coordinate of the cell.
    unsigned int x;

    // The Y coordinate of the cell.
    unsigned int y;

    // A value corresponding to the orientation.
    unsigned int angle_index;

    // Distinct types to make swapping the different indices harder.
    enum class LinearVelocity: int {};
    enum class AngularVelocity: int {};

    // A value (index) corresponding to the linear velocity.
    LinearVelocity linear_velocity;

    // A value (index) corresponding to the angular velocity.
    AngularVelocity angular_velocity;

    bool operator==(
        const Pose5D& other) const
    {
        const auto a = std::tie(x, y, angle_index, linear_velocity, angular_velocity);
        const auto b = std::tie(
            other.x, other.y, other.angle_index, other.linear_velocity, other.angular_velocity);

        return a == b;
    }

    friend bool operator<(
        const Pose5D& lh, const Pose5D& rh)
    {
        const auto a = std::tie(lh.x, lh.y, lh.angle_index, lh.linear_velocity, lh.angular_velocity);
        const auto b = std::tie(rh.x, rh.y, rh.angle_index, rh.linear_velocity, rh.angular_velocity);

        return a < b;
    }

    friend std::size_t hash_value(const Pose5D& p)
    {
        std::size_t seed = 0;

        boost::hash_combine(seed, p.x);
        boost::hash_combine(seed, p.y);
        boost::hash_combine(seed, p.angle_index);
        boost::hash_combine(seed, p.linear_velocity);
        boost::hash_combine(seed, p.angular_velocity);

        return seed;
    }
};

// A path from one pose to another. The first element is the start pose, the
// last element is the goal.
using Path = std::vector<Pose>;

// A path from one pose to another, as returned by the 5D planning algorithm.
using Path5D = std::vector<Pose5D>;

// Describes a heuristic for the 5D planner.
enum class HeuristicType {
    // The heuristic function always returns '0.0'.
    NONE,

    // Euclidean distance to the goal pose.
    EUCLIDEAN,

    // Manhattan distance to the goal pose.
    MANHATTAN,

    // The number of cells from the goal along the inflated 5D search space.
    DEPTH,

    // The number of cells from the goal of the closest element of the 3D path.
    PATH,
};

std::ostream& operator<<(
    std::ostream& os,
    HeuristicType ht);

std::ostream& operator<<(
    std::ostream& os,
    const Pose& pose);

std::ostream& operator<<(
    std::ostream& os,
    const Pose2D& pose);

std::ostream& operator<<(
    std::ostream& os,
    const Pose5D& pose);

}

#endif
