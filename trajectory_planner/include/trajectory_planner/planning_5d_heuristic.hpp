#ifndef TRAJECTORY_PLANNER_5D_HEURISTIC_HPP
#define TRAJECTORY_PLANNER_5D_HEURISTIC_HPP

#include "trajectory_planner/depth_heuristic.hpp"

namespace trajectory_planner::five
{

// The base class for heuristics used in the five dimensional search.
class Heuristic
{
public:
    virtual ~Heuristic() {};

    virtual float value(
        const Pose5D& pose) const = 0;
};

// A heuristic that is none, it always returns an estimated cost of
// zero, causing A* to perform liek Dijkstra.
class NoneHeuristic: public Heuristic
{
public:
    virtual ~NoneHeuristic() {};

    float value(
        const Pose5D& pose) const override;
};

// A heuristic that estimates the cost of a pose as the euclidean
// distance to the goal.
class EuclideanHeuristic: public Heuristic
{
    Pose5D goal_;

public:
    EuclideanHeuristic(
        Pose5D goal);

    virtual ~EuclideanHeuristic() {};

    float value(
        const Pose5D& pose) const override;
};

// A heuristic that estimates the cost of a pose as the manhattan
// distance to the goal.
class ManhattanHeuristic: public Heuristic
{
    Pose5D goal_;

public:
    ManhattanHeuristic(
        Pose5D goal);

    virtual ~ManhattanHeuristic() {};

    float value(
        const Pose5D& pose) const override;
};

// A heuristic that estimates the cost of a pose as the shortes
// cell distance in the search space inflated from the 3D search result.
class DepthHeuristic: public Heuristic
{
    trajectory_planner::DepthHeuristic heuristic_;

public:
    DepthHeuristic(
        Pose5D goal,
        const Costs& search_space);

    virtual ~DepthHeuristic() {};

    float value(
        const Pose5D& pose) const override;
};

// A heuristic that estimates the cost of a pose from the nearest element of
// the 3D path.
class PathHeuristic: public Heuristic
{
    // Pre-calculated map that contains an entry for each pose in the search space.
    std::unordered_map<Pose2D, float, boost::hash<Pose2D>> values_;

public:
    PathHeuristic(
        const Path& path,
        const Costs& search_space);

    virtual ~PathHeuristic() {};

    float value(
        const Pose5D& pose) const override;
};

}

#endif
