#include "trajectory_planner/depth_heuristic.hpp"

#include "trajectory_planner/planning_3d.hpp"

namespace trajectory_planner
{

DepthHeuristic::DepthHeuristic(
    const Costs& costs,
    const Pose2D& goal,
    bool costs_3d)
    : width_(costs.width())
    , height_(costs.height())
    , values_(width_ * height_, std::nan(""))
    , maximum_value_(0.0)
{
    // The nodes that are between the processed and unprocessed poses.
    std::vector<Pose2D> frontier;

    // At the start, the values for the goal poses is known.
    set_value(goal.x, goal.y, 0.0);
    frontier.push_back(goal);

    // The frontier of the last iteration.
    std::vector<Pose2D> old_frontier;

    while (!frontier.empty()) {
        // Candidates for the new frontier are all neighbours (direct or diagnoal)
        // of the current frontier set.
        std::vector<Pose2D> candidates;
        for (const Pose2D& p: frontier) {
            // The corner coordinates of a 3x3 region with the frontier pose in the
            // middle, capped to the limits of the map.
            size_t x_min = p.x > 1 ? p.x - 1 : p.x;
            size_t x_max = p.x < width_ - 1 ? p.x + 1 : p.x;
            size_t y_min = p.y > 1 ? p.y - 1 : p.y;
            size_t y_max = p.y < height_ - 1 ? p.y + 1 : p.y;

            // Add all neighbours to the current frontier cells into 'candidates'.
            for (size_t x = x_min; x <= x_max; x++) {
                for (size_t y = y_min; y <= y_max; y++) {
                    //candidates.insert(
                    candidates.emplace_back(
                        Pose2D{static_cast<unsigned int>(x), static_cast<unsigned int>(y)});
                }
            }
        }

        // Remove duplicates in the candidates vector.
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

        // Add poses from 'candidates' to the result if they are unprocessed.
        std::vector<Pose2D> new_frontier;
        for (const Pose2D& c: candidates) {
            if (!std::isnan(get_value(c.x, c.y))) {
                // There is already a heuristic value for this pose,
                // skip this candidate.
                continue;
            }

            // Check if there is a cost value for this cell.
            bool in_costs = false;

            if (costs_3d) {
                for (three::MovementIndex mi = 0; mi < three::movement_index_count; mi++) {
                    if (costs.get_3d_cost(c.x, c.y, mi) != Costs::invalid_3d_cost) {
                        in_costs = true;
                        break;
                    }
                }
            } else {
                for (unsigned int ai = 0; ai < costs.angle_granularity(); ai++) {
                    if (costs.get_5d_cost(c.x, c.y, ai)) {
                        in_costs = true;
                        break;
                    }
                }
            }

            if (!in_costs) {
                // This pose is not in the cost map, so skip it.
                continue;
            }

            // This is a valid candidate, we now process it, and it will be part
            // of the next iteration's frontier.
            new_frontier.push_back(c);

            // The corner coordinates of a 3x3 region with the candidate in the
            // middle, capped to the limits of the map.
            size_t x_min = c.x > 1 ? c.x - 1 : c.x;
            size_t x_max = c.x < width_ - 1 ? c.x + 1 : c.x;
            size_t y_min = c.y > 1 ? c.y - 1 : c.y;
            size_t y_max = c.y < height_ - 1 ? c.y + 1 : c.y;

            // Search for the shortest path into this cell.
            double smallest_value = std::numeric_limits<double>::max();
            for (size_t x = x_min; x <= x_max; x++) {
                for (size_t y = y_min; y <= y_max; y++) {
                    const double neighbour_value = get_value(x, y);
                    if (!std::isnan(neighbour_value)) {
                        // The distance going from the neighbour to the candidate
                        // is either "1.0" when the neighbour is on the same row
                        // or column, or "sqrt(2.0)" for diagonal neighbours.
                        const double dist = ((x == c.x) || (y == c.y)) ? 1.0 : sqrt(2.0);

                        // Store the smallest of all neighbouring values.
                        smallest_value = std::min(smallest_value, neighbour_value + dist);
                    }
                }
            }

            // Because there is at least one neighbour (a node in 'frontier' that
            // led us to 'candidate'), this value is not the maximum double value
            // any more. Also, we can end up here multiple times for the same pose
            // (the same pose as candidates from different elements in frontier),
            // but this is also fine as the already updated neighbour can not have
            // a smaller value as the frontier node, so this calculation will return
            // the same value in this case.
            set_value(c.x, c.y, smallest_value);

            // Also update the maximum value for all poses in the heuristic.
            maximum_value_ = std::max(maximum_value_, smallest_value);
        }

        old_frontier = std::move(frontier);
        frontier = std::move(new_frontier);
    }
}

double DepthHeuristic::get_value(
    unsigned int x,
    unsigned int y) const
{
    if (!(x < width_)) {
        std::ostringstream ss;
        ss
            << "DepthHeuristic::get_value(): x value of " << x
            << " is bigger than the map width of " << width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < height_)) {
        std::ostringstream ss;
        ss
            << "DepthHeuristic::get_value(): y value of " << y
            << " is bigger than the map height of " << height_ << ".";
        throw std::runtime_error(ss.str());
    }

    return values_.at(x + width_ * y);
}

void DepthHeuristic::set_value(
    unsigned int x,
    unsigned int y,
    double value)
{
    if (!(x < width_)) {
        std::ostringstream ss;
        ss
            << "DepthHeuristic::set_value(): x value of " << x
            << " is bigger than the map width of " << width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < height_)) {
        std::ostringstream ss;
        ss
            << "DepthHeuristic::set_value(): y value of " << y
            << " is bigger than the map height of " << height_ << ".";
        throw std::runtime_error(ss.str());
    }

    values_.at(x + width_ * y) = value;
}

}
