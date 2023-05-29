#include "trajectory_planner/costs.hpp"

#include <limits>

namespace trajectory_planner
{

Costs::Costs(
    size_t floorplan_width,
    size_t floorplan_height,
    unsigned int angle_granularity)
    : angle_granularity_(angle_granularity)
    , floorplan_width_(floorplan_width)
    , floorplan_height_(floorplan_height)
{
}

Costs::CostVector3D Costs::create_3d_cost_vector() const
{
    return CostVector3D(floorplan_width_ * floorplan_height_, invalid_3d_cost);
}

void Costs::set_3d_cost_vector(
    unsigned int movement_index,
    CostVector3D cost_vector)
{
    if (!(costs_3d_.size() > movement_index)) {
        costs_3d_.resize(movement_index + 1);
    }

    costs_3d_.at(movement_index) = std::move(cost_vector);
}

Costs::CostType3D Costs::get_3d_cost(
    unsigned int x,
    unsigned int y,
    unsigned int movement_index) const
{
    if (!(x < floorplan_width_)) {
        std::ostringstream ss;
        ss
            << "Costs::get_3d_cost(): x value of " << x
            << " is bigger than floorplan_width_ of " << floorplan_width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < floorplan_height_)) {
        std::ostringstream ss;
        ss
            << "Costs::get_3d_cost(): y value of " << y
            << " is bigger than floorplan_height_ of " << floorplan_height_ << ".";
        throw std::runtime_error(ss.str());
    }

    return costs_3d_.at(movement_index).at(x + floorplan_width_ * y);
}

void Costs::set_3d_cost(
    CostVector3D& cost_vector,
    unsigned int x,
    unsigned int y,
    float cost) const
{
    if (!(x < floorplan_width_)) {
        std::ostringstream ss;
        ss
            << "Costs::set_3d_cost(): x value of " << x
            << " is bigger than floorplan_width_ of " << floorplan_width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < floorplan_height_)) {
        std::ostringstream ss;
        ss
            << "Costs::set_3d_cost(): y value of " << y
            << " is bigger than floorplan_height_ of " << floorplan_height_ << ".";
        throw std::runtime_error(ss.str());
    }

    // Convert float value to the type used to store the cost values.
    float capped_cost = std::max(0.0f, std::min(1.0f, cost));
    CostType3D c = (invalid_3d_cost - 1) * capped_cost;

    cost_vector.at(x + floorplan_width_ * y) = c;
}

Costs::CostVector5D Costs::create_5d_cost_vector() const
{
    return CostVector5D(floorplan_width_ * floorplan_height_, false);
}

void Costs::set_5d_cost_vector(
    unsigned int angle_index,
    CostVector5D cost_vector)
{
    if (!(costs_5d_.size() > angle_index)) {
        costs_5d_.resize(angle_index + 1);
    }

    costs_5d_.at(angle_index) = std::move(cost_vector);
}

bool Costs::get_5d_cost(
    unsigned int x,
    unsigned int y,
    unsigned int angle_index) const
{
    if (!(x < floorplan_width_)) {
        std::ostringstream ss;
        ss
            << "Costs::get_5d_cost(): x value of " << x
            << " is bigger than floorplan_width_ of " << floorplan_width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < floorplan_height_)) {
        std::ostringstream ss;
        ss
            << "Costs::get_5d_cost(): y value of " << y
            << " is bigger than floorplan_height_ of " << floorplan_height_ << ".";
        throw std::runtime_error(ss.str());
    }

    return costs_5d_.at(angle_index).at(x + floorplan_width_ * y);
}

void Costs::set_5d_cost(
    CostVector5D& cost_vector,
    unsigned int x,
    unsigned int y) const
{
    if (!(x < floorplan_width_)) {
        std::ostringstream ss;
        ss
            << "Costs::set_5d_cost(): x value of " << x
            << " is bigger than floorplan_width_ of " << floorplan_width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < floorplan_height_)) {
        std::ostringstream ss;
        ss
            << "Costs::set_5d_cost(): y value of " << y
            << " is bigger than floorplan_height_ of " << floorplan_height_ << ".";
        throw std::runtime_error(ss.str());
    }

    cost_vector.at(x + floorplan_width_ * y) = true;
}

unsigned int Costs::angle_granularity() const
{
    return angle_granularity_;
}

Costs Costs::intersect_5d_costs(
    std::vector<Pose2D> poses) const
{
    Costs result(
        floorplan_width_, floorplan_height_, angle_granularity_);

    for (unsigned int ai = 0; ai < angle_granularity_; ai++) {
        result.set_5d_cost_vector(ai, result.create_5d_cost_vector());
    }

    for (const Pose2D& pose: poses) {
        for (unsigned int ai = 0; ai < angle_granularity_; ai++) {
            if (get_5d_cost(pose.x, pose.y, ai)) {
                result.set_5d_cost(result.costs_5d_.at(ai), pose.x, pose.y);
            }
        }
    }

    return result;
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

}
