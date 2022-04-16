#include "trajectory_planner/costs.hpp"

#include <limits>

namespace trajectory_planner
{

const double Costs::invalid_cost = -1.0;

const Costs::CostType Costs::internal_invalid_cost =
    std::numeric_limits<Costs::CostType>::max();

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

void Costs::set_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index,
    double cost)
{
    auto it = costs_.find(std::make_pair(x, y));
    if (it == costs_.end()) {
        // There is no entry for this X/Y location. Add a vector with all
        // orientations, except the given one, set to the invalid value.
        CostVector cv = CostVector(angle_granularity_, internal_invalid_cost);
        cv.at(angle_index) = double_cost_to_internal_cost(cost);
        costs_[std::make_pair(x, y)] = cv;
    } else {
        // There is already a cost vector for this X/Y location, update it.
        it->second.at(angle_index) = double_cost_to_internal_cost(cost);
    }
}

double Costs::get_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index) const
{
    auto it = costs_.find(std::make_pair(x, y));
    if (it == costs_.end()) {
        // No entry for this X/Y location.
        return invalid_cost;
    }

    // Return the cost value for the given orientation.
    return internal_cost_to_double_cost(it->second.at(angle_index));
}

double Costs::get_cost(
    const Pose& pose) const
{
    return get_cost(pose.x, pose.y, pose.angle_index);
}

unsigned int Costs::angle_granularity() const
{
    return angle_granularity_;
}

unsigned int Costs::goal_turn_penalty_distance() const
{
    return goal_turn_penalty_distance_;
}

double Costs::internal_cost_to_double_cost(
    const CostType c) const
{
    if (c == internal_invalid_cost) {
        return invalid_cost;
    }

    return static_cast<double>(c) / static_cast<double>(internal_invalid_cost - 1);
}

Costs::CostType Costs::double_cost_to_internal_cost(
    const double c) const
{
    return (internal_invalid_cost - 1) * c;
}

}
