#include "trajectory_planner/costs.hpp"

#include <limits>

namespace trajectory_planner
{

const double Costs::invalid_cost_3d = -1.0;

const Costs::CostType3D Costs::internal_invalid_cost_3d =
    std::numeric_limits<Costs::CostType3D>::max();

Costs::Costs(
    unsigned int angle_granularity,
    unsigned int goal_turn_penalty_distance)
    : angle_granularity_(angle_granularity)
    , goal_turn_penalty_distance_(goal_turn_penalty_distance)
{
    if (0 == goal_turn_penalty_distance_) {
        throw std::runtime_error("Goal turn penalty distance is zero.");
    }
}

void Costs::set_3d_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index,
    double cost)
{
    auto it = costs_3d_.find(std::make_pair(x, y));
    if (it == costs_3d_.end()) {
        // There is no entry for this X/Y location. Add a vector with all
        // orientations, except the given one, set to the invalid value.
        CostVector3D cv = CostVector3D(angle_granularity_, internal_invalid_cost_3d);
        cv.at(angle_index) = double_cost_to_internal_cost_3d(cost);
        costs_3d_[std::make_pair(x, y)] = cv;
    } else {
        // There is already a cost vector for this X/Y location, update it.
        it->second.at(angle_index) = double_cost_to_internal_cost_3d(cost);
    }
}

double Costs::get_3d_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index) const
{
    auto it = costs_3d_.find(std::make_pair(x, y));
    if (it == costs_3d_.end()) {
        // No entry for this X/Y location.
        return invalid_cost_3d;
    }

    // Return the cost value for the given orientation.
    return internal_cost_3d_to_double_cost(it->second.at(angle_index));
}

double Costs::get_3d_cost(
    const Pose& pose) const
{
    return get_3d_cost(pose.x, pose.y, pose.angle_index);
}

unsigned int Costs::angle_granularity() const
{
    return angle_granularity_;
}

unsigned int Costs::goal_turn_penalty_distance() const
{
    return goal_turn_penalty_distance_;
}

Costs Costs::intersect(
    std::vector<Pose2D> poses) const
{
    Costs result(angle_granularity_, goal_turn_penalty_distance_);

    for (const Pose2D& pose: poses) {
        const Location location(pose.x, pose.y);

        auto it = costs_3d_.find(location);
        if (it != costs_3d_.end()) {
            result.costs_3d_[location] = it->second;
        }
    }

    return result;
}

void Costs::export_poses(
    std::unordered_set<Pose2D, boost::hash<Pose2D>>& poses_set) const
{
    for (const auto& it: costs_3d_) {
        poses_set.insert(Pose2D{std::get<0>(it.first), std::get<1>(it.first)});
    }
}

double Costs::internal_cost_3d_to_double_cost(
    const CostType3D c) const
{
    if (c == internal_invalid_cost_3d) {
        return invalid_cost_3d;
    }

    return static_cast<double>(c) / static_cast<double>(internal_invalid_cost_3d - 1);
}

Costs::CostType3D Costs::double_cost_to_internal_cost_3d(
    const double c) const
{
    return (internal_invalid_cost_3d - 1) * c;
}

}
