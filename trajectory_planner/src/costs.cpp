#include "trajectory_planner/costs.hpp"

#include <limits>

namespace trajectory_planner
{

const double Costs::invalid_cost_3d = -1.0;

const Costs::CostType3D Costs::internal_invalid_cost_3d =
    std::numeric_limits<Costs::CostType3D>::max();

Costs::Costs(
    size_t floorplan_width,
    size_t floorplan_height,
    unsigned int angle_granularity,
    unsigned int goal_turn_penalty_distance)
    : angle_granularity_(angle_granularity)
    , floorplan_width_(floorplan_width)
    , floorplan_height_(floorplan_height)
    , goal_turn_penalty_distance_(goal_turn_penalty_distance)
    , costs_5d_(initialize_5d_costs(floorplan_width, floorplan_height, angle_granularity))
{
    if (0 == goal_turn_penalty_distance_) {
        throw std::runtime_error("Goal turn penalty distance is zero.");
    }
}

void Costs::set_3d_cost(
    unsigned int x,
    unsigned int y,
    unsigned int movement_index,
    double cost)
{
    auto it = costs_3d_.find(std::make_pair(x, y));
    if (it == costs_3d_.end()) {
        // There is no entry for this X/Y location. Add a vector with all
        // orientations, except the given one, set to the invalid value.
        CostVector3D cv = CostVector3D(angle_granularity_, internal_invalid_cost_3d);
        cv.at(movement_index) = double_cost_to_internal_cost_3d(cost);
        costs_3d_[std::make_pair(x, y)] = cv;
    } else {
        // There is already a cost vector for this X/Y location, update it.
        it->second.at(movement_index) = double_cost_to_internal_cost_3d(cost);
    }
}

double Costs::get_3d_cost(
    unsigned int x,
    unsigned int y,
    unsigned int movement_index) const
{
    auto it = costs_3d_.find(std::make_pair(x, y));
    if (it == costs_3d_.end()) {
        // No entry for this X/Y location.
        return invalid_cost_3d;
    }

    // Return the cost value for the given orientation.
    return internal_cost_3d_to_double_cost(it->second.at(movement_index));
}

void Costs::set_5d_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index)
{
    costs_5d_.at(angle_index).at(x + floorplan_width_ * y) = true;
}

bool Costs::get_5d_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index) const
{
    return costs_5d_.at(angle_index).at(x + floorplan_width_ * y);
}

unsigned int Costs::angle_granularity() const
{
    return angle_granularity_;
}

unsigned int Costs::goal_turn_penalty_distance() const
{
    return goal_turn_penalty_distance_;
}

Costs Costs::intersect_5d_costs(
    std::vector<Pose2D> poses) const
{
    Costs result(
        floorplan_width_, floorplan_height_, angle_granularity_, goal_turn_penalty_distance_);

    for (const Pose2D& pose: poses) {
        for (unsigned int ai = 0; ai < angle_granularity_; ai++) {
            if (get_5d_cost(pose.x, pose.y, ai)) {
                result.set_5d_cost(pose.x, pose.y, ai);
            }
        }
    }

    return result;
}

void Costs::export_3d_poses(
    std::unordered_set<Pose2D, boost::hash<Pose2D>>& poses_set) const
{
    for (const auto& it: costs_3d_) {
        poses_set.insert(Pose2D{std::get<0>(it.first), std::get<1>(it.first)});
    }
}

void Costs::export_5d_poses(
    std::unordered_set<Pose2D, boost::hash<Pose2D>>& poses_set) const
{
    for (unsigned int x = 0; x < floorplan_width_; x++) {
        for (unsigned int y = 0; y < floorplan_height_; y++) {
            bool b = false;

            for (const auto& v: costs_5d_) {
                if (v.at(x + floorplan_width_ * y)) {
                    b = true;
                    break;
                }
            }

            if (b) {
                poses_set.insert(Pose2D{x, y});
            }
        }
    }
}

std::vector<Costs::CostVector5D> Costs::initialize_5d_costs(
    size_t width,
    size_t height,
    unsigned int angle_granularity) const
{
    std::vector<CostVector5D> result(angle_granularity);

    for (CostVector5D& v: result) {
        v.resize(width * height, false);
    }

    return result;
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
