#include "trajectory_planner/turn_costs.hpp"

namespace trajectory_planner
{

TurnCosts::TurnCosts(
    const DepthHeuristic& heuristic,
    const float goal_distance)
    : width_(heuristic.width())
    , height_(heuristic.height())
    , turn_costs_(width_ * height_, std::nanf(""))
{
    for (size_t x = 0; x < width_; x++) {
        for (size_t y = 0; y < height_; y++) {
            const float dist = heuristic.get_value(x, y);

            if (std::isnan(dist)) {
                continue;
            }

            float cost = expf(-3.0f * dist / goal_distance);
            cost = std::min(1.0f, cost);
            cost = std::max(0.0f, cost);

            set_value(x, y, cost);
        }
    }
}

size_t TurnCosts::width() const
{
    return width_;
}

size_t TurnCosts::height() const
{
    return height_;
}

float TurnCosts::get_value(
    unsigned int x,
    unsigned int y) const
{
    if (!(x < width_)) {
        std::ostringstream ss;
        ss
            << "TurnCosts::get_value(): x value of " << x
            << " is bigger than the map width of " << width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < height_)) {
        std::ostringstream ss;
        ss
            << "TurnCosts::get_value(): y value of " << y
            << " is bigger than the map height of " << height_ << ".";
        throw std::runtime_error(ss.str());
    }

    return turn_costs_.at(x + width_ * y);
}

void TurnCosts::set_value(
    unsigned int x,
    unsigned int y,
    float value)
{
    if (!(x < width_)) {
        std::ostringstream ss;
        ss
            << "TurnCosts::set_value(): x value of " << x
            << " is bigger than the map width of " << width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < height_)) {
        std::ostringstream ss;
        ss
            << "TurnCosts::set_value(): y value of " << y
            << " is bigger than the map height of " << height_ << ".";
        throw std::runtime_error(ss.str());
    }

    turn_costs_.at(x + width_ * y) = value;
}

}
