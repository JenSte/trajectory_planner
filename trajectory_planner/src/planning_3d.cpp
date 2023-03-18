#include "trajectory_planner/planning_3d.hpp"
#include "trajectory_planner/a_star.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace {
    // This value is multiplied with the (normalized) angle difference when turning
    // on the spot. A turn of 180 degrees will correspond to the full 'angle_factor'.
    const double angle_factor = 10.0;
}

namespace trajectory_planner::three
{

HeuristicMap::HeuristicMap(
    const Costs& costs,
    const Pose3D& goal)
    : width_(costs.width())
    , height_(costs.height())
    , values_(width_ * height_, std::nan(""))
{
    // The list of nodes that are between the processed and unprocessed poses.
    std::unordered_set<Pose2D, boost::hash<Pose2D>> frontier;

    // The set of all unprocessed poses.
    std::unordered_set<Pose2D, boost::hash<Pose2D>> remaining;
    costs.export_3d_poses(remaining);

    // At the start, the values for the goal poses is known.
    set_value(goal.x, goal.y, 0.0);
    frontier.insert(Pose2D{goal.x, goal.y});
    remaining.erase(Pose2D{goal.x, goal.y});

    // The frontier of the last iteration.
    std::unordered_set<Pose2D, boost::hash<Pose2D>> old_frontier;

    // The number of iterations the loop below already ran, also the current
    // distance to the goal.
    unsigned int iteration = 0;

    for (;;) {
        iteration++;

        // Candidates for the new frontier are all neighbours (direct or diagnoal)
        // of the current frontier set.
        std::unordered_set<Pose2D, boost::hash<Pose2D>> candidates;
        for (const Pose2D& p: frontier) {
            candidates.insert(Pose2D{p.x + 1, p.y + 1});
            candidates.insert(Pose2D{p.x + 1, p.y});
            candidates.insert(Pose2D{p.x + 1, p.y - 1});
            candidates.insert(Pose2D{p.x, p.y - 1});
            candidates.insert(Pose2D{p.x - 1, p.y - 1});
            candidates.insert(Pose2D{p.x - 1, p.y});
            candidates.insert(Pose2D{p.x - 1, p.y + 1});
            candidates.insert(Pose2D{p.x, p.y + 1});
        }

        // Don't look back, remove all the elements that
        // were processed in the last iterations.
        for (auto it = candidates.begin(); it != candidates.end();) {
            if (frontier.find(*it) != frontier.end()) {
                it = candidates.erase(it);
            } else if (old_frontier.find(*it) != old_frontier.end()) {
                it = candidates.erase(it);
            } else {
                it++;
            }
        }

        // Add poses from 'candidates' to the result if they are unprocessed.
        std::unordered_set<Pose2D, boost::hash<Pose2D>> new_frontier;
        for (const Pose2D& c: candidates) {
            if (remaining.erase(c) == 1) {
                // If 'remaining.erase()' returns 1, the candidate 'c' was in remaining.
                set_value(c.x, c.y, iteration);
                new_frontier.insert(c);
            }
        }

        if (new_frontier.empty()) {
            // No more new cells that are next to the current frontier, done.
            break;
        }

        old_frontier = std::move(frontier);
        frontier = std::move(new_frontier);
    }
}

double HeuristicMap::get_value(
    unsigned int x,
    unsigned int y) const
{
    if (!(x < width_)) {
        std::ostringstream ss;
        ss
            << "HeuristicMap::get_value(): x value of " << x
            << " is bigger than the map width of " << width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < height_)) {
        std::ostringstream ss;
        ss
            << "HeuristicMap::get_value(): y value of " << y
            << " is bigger than the map height of " << height_ << ".";
        throw std::runtime_error(ss.str());
    }

    return values_.at(x + width_ * y);
}

void HeuristicMap::set_value(
    unsigned int x,
    unsigned int y,
    double value)
{
    if (!(x < width_)) {
        std::ostringstream ss;
        ss
            << "HeuristicMap::set_value(): x value of " << x
            << " is bigger than the map width of " << width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < height_)) {
        std::ostringstream ss;
        ss
            << "HeuristicMap::set_value(): y value of " << y
            << " is bigger than the map height of " << height_ << ".";
        throw std::runtime_error(ss.str());
    }

    values_.at(x + width_ * y) = value;
}

std::tuple<Pose3D, Pose3D> linear_neighbours(
    const Pose3D& pose)
{
    // The calculation of the index without a check may over/undflow, which would
    // make the cell with a coordinate of 0 a neighbour of the cell with the
    // coordinate UINT_MAX, but I guess in that case there are other problems...

    switch (pose.movement) {
        case 0:
        case 8:
            return std::make_tuple(
                Pose3D{pose.x + 1, pose.y, pose.movement},
                Pose3D{pose.x - 1, pose.y, pose.movement});

        case 1:
        case 9:
            return std::make_tuple(
                Pose3D{pose.x + 2, pose.y + 1, pose.movement},
                Pose3D{pose.x - 2, pose.y - 1, pose.movement});

        case 2:
        case 10:
            return std::make_tuple(
                Pose3D{pose.x + 1, pose.y + 1, pose.movement},
                Pose3D{pose.x - 1, pose.y - 1, pose.movement});

        case 3:
        case 11:
            return std::make_tuple(
                Pose3D{pose.x + 1, pose.y + 2, pose.movement},
                Pose3D{pose.x - 1, pose.y - 2, pose.movement});

        case 4:
        case 12:
            return std::make_tuple(
                Pose3D{pose.x, pose.y + 1, pose.movement},
                Pose3D{pose.x, pose.y - 1, pose.movement});

        case 5:
        case 13:
            return std::make_tuple(
                Pose3D{pose.x - 1, pose.y + 2, pose.movement},
                Pose3D{pose.x + 1, pose.y - 2, pose.movement});

        case 6:
        case 14:
            return std::make_tuple(
                Pose3D{pose.x - 1, pose.y + 1, pose.movement},
                Pose3D{pose.x + 1, pose.y - 1, pose.movement});

        case 7:
        case 15:
            return std::make_tuple(
                Pose3D{pose.x - 2, pose.y + 1, pose.movement},
                Pose3D{pose.x + 2, pose.y - 1, pose.movement});
    }

    throw std::runtime_error("Movement index out of range.");
}

std::tuple<Pose3D, Pose3D> turn_neighbours(
    const Pose3D& pose)
{
    return std::make_tuple(
        Pose3D{pose.x, pose.y, (pose.movement + 1) % movement_index_count},
        Pose3D{pose.x, pose.y, (pose.movement - 1) % movement_index_count});
}

std::tuple<Pose3D, Pose3D, Pose3D, Pose3D> move_turn_neighbours(
    const std::tuple<Pose3D, Pose3D>& ln)
{
    return std::make_tuple(
        Pose3D{std::get<0>(ln).x, std::get<0>(ln).y, (std::get<0>(ln).movement + 1) % movement_index_count},
        Pose3D{std::get<0>(ln).x, std::get<0>(ln).y, (std::get<0>(ln).movement - 1) % movement_index_count},
        Pose3D{std::get<1>(ln).x, std::get<1>(ln).y, (std::get<1>(ln).movement + 1) % movement_index_count},
        Pose3D{std::get<1>(ln).x, std::get<1>(ln).y, (std::get<1>(ln).movement - 1) % movement_index_count});
}

boost::container::static_vector<Pose3D, 8> neighbours(
    const Costs& costs,
    const Pose3D& pose)
{
    boost::container::static_vector<Pose3D, 8> result;

    auto add_neighbour = [&result, &costs](Pose3D p) {
        const Costs::CostType3D cost = costs.get_3d_cost(p.x, p.y, p.movement);
        if (cost != Costs::invalid_3d_cost) {
            // This pose is in the costs object.
            result.emplace_back(std::move(p));
        }
    };

    std::tuple<Pose3D, Pose3D> ln = linear_neighbours(pose);
    add_neighbour(std::get<0>(ln));
    add_neighbour(std::get<1>(ln));

    std::tuple<Pose3D, Pose3D> tn = turn_neighbours(pose);
    add_neighbour(std::get<0>(tn));
    add_neighbour(std::get<1>(tn));

    std::tuple<Pose3D, Pose3D, Pose3D, Pose3D> mt = move_turn_neighbours(ln);
    add_neighbour(std::get<0>(mt));
    add_neighbour(std::get<1>(mt));
    add_neighbour(std::get<2>(mt));
    add_neighbour(std::get<3>(mt));

    return result;
}

Pose3D refine_pose(
    const Costs& costs,
    const Pose& pose)
{
    const double angle = pose.angle_index * (2 * M_PI) / costs.angle_granularity();

    MovementIndex movement_index =
        static_cast<unsigned int>(
            std::round(angle / ((2 * M_PI) / movement_index_count)));

    // When the angle is close to 2 * pi, it is rounded up to an invalid index.
    movement_index %= movement_index_count;

    // Make sure the new pose, with an orientation rounded to a
    // more granular value is also valid.
    const Costs::CostType3D cost = costs.get_3d_cost(pose.x, pose.y, movement_index);
    if (cost == Costs::invalid_3d_cost) {
        std::ostringstream ss;
        ss
            << std::setprecision(2)
            << "Refined 3D pose is not valid ("
            << "angle_index = " << pose.angle_index << ", "
            << "angle = " << angle << " rad, "
            << "movement_index = " << movement_index << ")";
        throw std::runtime_error(ss.str());
    }

    return Pose3D{pose.x, pose.y, movement_index};
}

double normalized_angle_distance(
    const unsigned int movement_index_a,
    const unsigned int movement_index_b)
{
    // The actual orientation angles in radians.
    const double a = angle_lut[movement_index_a];
    const double b = angle_lut[movement_index_b];

    // The difference between the two orientations, from -pi to pi.
    const double diff = atan2(sin(a - b), cos(a - b));

    // We return the difference between the two orientations, normalized to
    // the range from 0.0 to 1.0.
    return abs(diff) / M_PI;
}

double calculate_movemement_cost(
    const Costs& costs,
    const Pose3D& goal_pose,
    const Pose3D& from,
    const Pose3D& to)
{
    // Calculate the cost of a pure forward/backward movement.
    auto pure_movement_cost = [](const Pose3D& from, const Pose3D& to, double cost_value) {
        // The distance when moving from 'from' to 'to'.
        const double dx = static_cast<double>(from.x) - static_cast<double>(to.x);
        const double dy = static_cast<double>(from.y) - static_cast<double>(to.y);
        const double distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));

        // The cost of the neighbour is multiplied by the distance to the cell,
        // so that edges that move diagonally do not have an advantage.
        return distance * (1.0 + cost_value);
    };

    // Calculate the costs of turning on the spot.
    auto pure_turn_cost = [&costs, &goal_pose](const Pose3D& from, const Pose3D& to, double cost_value) {
        // Turning on the spot. We calculate the distance to the goal, and apply
        // a penalty for poses close to the goal. This is so that turns near the
        // goal are discouraged as turning often causes the robot to deviate from
        // the pose and makes it harder to hit the goal exactly.
        const double dx = static_cast<double>(goal_pose.x) - static_cast<double>(to.x);
        const double dy = static_cast<double>(goal_pose.y) - static_cast<double>(to.y);
        const double goal_distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));

        double goal_penalty = 1.0 + exp(-goal_distance / costs.goal_turn_penalty_distance());
        if (goal_distance < 1.0) {
            goal_penalty = 2.0;
        }

        const double angle_distance = normalized_angle_distance(from.movement, to.movement);

        return goal_penalty * (1.0 + angle_factor * angle_distance) * (1.0 + cost_value);
    };

    // The value on the costmap at the 'to' pose.
    const double to_cost = Costs::cost_3d_to_double(
        costs.get_3d_cost(to.x, to.y, to.movement));

    if (from.movement == to.movement) {
        // The orientation stayed the same: pure forward/backward movement.
        return pure_movement_cost(from, to, to_cost);
    } else {
        if ((from.x == to.x) && (from.y == to.y)) {
            // Turning on the spot.
            return pure_turn_cost(from, to, to_cost);
        } else {
            // Moving forward/backward and turning at the same time.

            // The pose "between" 'from' and 'to'.
            Pose3D via{to.x, to.y, from.movement};

            double mc = pure_movement_cost(from, via, to_cost);
            double tc = pure_turn_cost(via, to, to_cost);

            // Calculate the cost of the combined "move + turn" from the costs
            // of separate "pure move" and "pure turn" costs when going over 'via'.
            // The turn cost is half the cost of a pure turn, this way multiple
            // consecutive turns are still "expensive", while only a single turn
            // can be cone relatively cheaply by using such a combined move. Also,
            // this cost value honors the triangle inequality.
            return mc + 0.5 * tc;
        }
    }
}

double calculate_heuristic(
    const HeuristicMap& heuristic_map,
    const Pose3D& goal_pose,
    const Pose3D& pose)
{
    double goal_distance = heuristic_map.get_value(pose.x, pose.y);
    if (std::isnan(goal_distance)) {
        // A fallback if the pose is not in 'heuristic_map'. In this case there is
        // probably no path to the goal in the search space (otherwise the heuristic
        // map calculation would include the pose), so we could also detect this earlier.
        const double dx = static_cast<double>(goal_pose.x) - static_cast<double>(pose.x);
        const double dy = static_cast<double>(goal_pose.y) - static_cast<double>(pose.y);
        goal_distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
    }

    // Calculate the normalized angle differenct between the orientation of the
    // pose and the goal pose.
    const double angle_distance = normalized_angle_distance(pose.movement, goal_pose.movement);

    // Compare to 'pure_movement_cost()': If we drive only forward, without ever
    // turning, far from every obstacle, so that the cost of a cell is 0.0:
    const double movement_cost = goal_distance;

    // Compare to 'pure_turn_cost()', but also to the calculation of the combined
    // "move + turn" step (taking only half the costs of a pure turn). No 'goal_penalty'
    // (see 'pure_turn_cost()' is applied, as we can not know where the turns would occur).
    const double turn_cost = 0.5 * (angle_factor * angle_distance);

    return movement_cost + turn_cost;
}

SearchResult3D plan(
    const Costs& costs,
    const Pose& start,
    const Pose& goal)
{
    // Convert the poses for the trajectory planner to the (more granular
    // in regard to the orientation) poses used here.
    Pose3D start_pose = refine_pose(costs, start);
    Pose3D goal_pose = refine_pose(costs, goal);

    // The main heuristic for the 3D search is the distance to the goal, calculated
    // by flood-filling the 3D search space in 2D. While it takes some time to
    // pre-calculate these values before doing the actual search, it pays of on
    // large maps, and does not take much time on small maps.
    HeuristicMap heuristic_map(costs, goal_pose);

    auto goal_reached = [&goal_pose](const Pose3D& pose) {
        return goal_pose == pose;
    };

    auto get_neighbours = [&costs](const Pose3D& pose) {
        return neighbours(costs, pose);
    };

    auto movement_cost = [&costs, &goal_pose](const Pose3D& pose, const Pose3D& neighbour) {
        return calculate_movemement_cost(costs, goal_pose, pose, neighbour);
    };

    auto heuristic = [&costs, &goal_pose, &heuristic_map](const Pose3D& pose) {
        return calculate_heuristic(heuristic_map, goal_pose, pose);
    };

    using a_star_type = AStar<
        Pose3D,
        decltype(goal_reached),
        decltype(get_neighbours),
        decltype(movement_cost),
        decltype(heuristic),
        true>;

    a_star_type a_star;
    a_star_type::search_result a_star_result = a_star.search(
        goal_reached,
        get_neighbours,
        movement_cost,
        heuristic,
        start_pose);

    SearchResult3D result(std::move(heuristic_map));

    // Convert the 3D path back to a path for the trajectory planner
    // and create the heuristic vector.
    const Path3D& path = std::get<0>(a_star_result);
    for (const Pose3D& pose: path) {
        const double angle = angle_lut[pose.movement];

        int angle_index = angle / (2 * M_PI / costs.angle_granularity());
        angle_index %= costs.angle_granularity();

        result.path.push_back(Pose{pose.x, pose.y, static_cast<unsigned int>(angle_index)});
        result.heuristic.push_back(result.heuristic_map.get_value(pose.x, pose.y));
    }

    // Create the costs vector in the result. This is done by iterating over
    // pairs in the found path and summing up the cost values along the way.
    result.cost.push_back(0.0); // Cost of the goal.

    double cost = 0.0;
    for (size_t i = path.size(); i-- > 1;) {
        const Pose3D& node = path.at(i);
        const Pose3D& predecessor = path.at(i - 1);
        cost += movement_cost(predecessor, node);
        result.cost.push_back(cost);
    }
    std::reverse(result.cost.begin(), result.cost.end());

    // Count the orientations for all cells visited.
    for (const Pose3D& node: std::get<1>(a_star_result)) {
        auto it = result.opened_nodes.find(Pose2D{node.x, node.y});
        if (it == result.opened_nodes.end()) {
            // The first time we come accross this X/Y coordinate.
            result.opened_nodes[Pose2D{node.x, node.y}] = 1;
        } else {
            // This coordinate is already known to the resulting map.
            it->second++;
        }
    }

    return result;
}

}
