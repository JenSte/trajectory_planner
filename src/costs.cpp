#include "trajectory_planner/costs.hpp"

namespace trajectory_planner
{

const double Costs::invalid_cost = -1.0;

Costs::Costs(
    unsigned int angle_granularity)
    : angle_granularity_(angle_granularity)
{
}

void Costs::set_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index,
    double cost)
{
    auto it = costs_.find(std::make_tuple(x, y));
    if (it == costs_.end()) {
        // There is no entry for this X/Y location. Add a vector with all
        // orientations, except the given one, set to the invalid value.
        CostVector cv = CostVector(angle_granularity_, invalid_cost);
        cv.at(angle_index) = cost;
        costs_[std::make_tuple(x, y)] = cv;
    } else {
        // There is already a cost vector for this X/Y location, update it.
        it->second.at(angle_index) = cost;
    }
}

double Costs::get_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index) const
{
    auto it = costs_.find(std::make_tuple(x, y));
    if (it == costs_.end()) {
        // No entry for this X/Y location.
        return invalid_cost;
    }

    // Return the cost value for the given orientation.
    return it->second.at(angle_index);
}

unsigned int Costs::angle_granularity() const
{
    return angle_granularity_;
}

}
