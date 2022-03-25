#ifndef TRAJECTORY_PLANNER_TYPES_HPP
#define TRAJECTORY_PLANNER_TYPES_HPP

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
};

// A path from one pose to another. The first element is the start pose, the
// last element is the goal.
using Path = std::vector<Pose>;

}

#endif
