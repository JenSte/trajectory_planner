#include "trajectory_planner/planning_3d.hpp"
#include "trajectory_planner/a_star.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace {
    // This value is multiplied with the (normalized) angle difference when turning
    // on the spot. A turn of 180 degrees will correspond to the full 'angle_factor'.
    const float angle_factor = 10.0f;
}

namespace trajectory_planner::three
{

// Stores the total costs along a path to a cell during an A* search.
class TotalCostsMap
{
    // Used to hold the total costs for the 16 different orientations of a cell.
    using cost_array = std::array<float, movement_index_count>;

public:

    // Initializes a new 'TotalCostsMap' object.
    TotalCostsMap(
        size_t width,
        size_t height)
        : width_(width)
        , height_(height)
        , total_costs_(width * height)
    {
        // The std::array<> used to hold the total costs for each cell does
        // not have a constructor, so we have to do the initialization this way.
        for (cost_array& cv: total_costs_) {
            for (float& f: cv) {
                f = std::numeric_limits<float>::infinity();
            }
        }
    }

    // Set the total cost value for a pose.
    void set(const Pose3D& pose, float cost)
    {
        total_costs_.at(pose.x + width_ * pose.y).at(pose.movement) = cost;
    }

    // Return the total cost to the pose, or infinity if the pose is not in the object.
    float get(const Pose3D& pose) const
    {
        return total_costs_.at(pose.x + width_ * pose.y).at(pose.movement);
    }

    // Count and return the number a 2D cell was looked at during the search.
    SearchResult3D::OpenedNodesMap count_opened_nodes() const
    {
        SearchResult3D::OpenedNodesMap result;

        for (size_t x = 0; x < width_; x++) {
            for (size_t y = 0; y < height_; y++) {
                // Count the number of cost values that were modified at least once
                // (are not infinite any more) in the cost array for this cell.
                const cost_array& vc = total_costs_.at(x + width_ * y);
                unsigned int c = std::count_if(vc.begin(), vc.end(), [](float f) {
                    return std::isfinite(f);
                });

                // Only cells where at least one orientation was looked at are included
                // in the result set.
                if (c != 0) {
                    result[Pose2D{static_cast<unsigned>(x), static_cast<unsigned>(y)}] = c;
                }
            }
        }

        return result;
    }

private:

    // The width of the map this object holds the total costs for.
    size_t width_;

    // The height of the map this object holds the total costs for.
    size_t height_;

    // Holds the total costs to a given node from the start node of a search.
    std::vector<cost_array> total_costs_;
};

// Stores the predecessor for a pose in the A* search.
class PredecessorsMap
{
public:

    // Initializes a new 'PredecessorMap' object.
    PredecessorsMap(
        size_t width,
        size_t height)
        : width_(width)
        , height_(height)
        , predecessors_(width * height)
    {
    }

    // Set the predecessor of a pose.
    void set(
        const Pose3D& pose,
        const Pose3D& predecessor)
    {
        // Because the 3D planner can only "move" a very limited distance in
        // the search space in one step (+/- 2 for X/Y coordinates and +/- 1
        // for the movement value), we pack the delta values to the predecessor
        // pose into one byte and store the information this way.

        int dx = static_cast<int>(pose.x) - static_cast<int>(predecessor.x);
        int dy = static_cast<int>(pose.y) - static_cast<int>(predecessor.y);
        int dm = static_cast<int>(pose.movement) - static_cast<int>(predecessor.movement);

        dx += 3;
        dy += 3;
        dm += 1;

        uint8_t b = (dm << 6) | (dy << 3) | dx;

        predecessors_.at(pose.x + width_ * pose.y).at(pose.movement) = b;
    }

    // Get the predecessor of a pose.
    Pose3D get(
        const Pose3D& pose) const
    {
        uint8_t b = predecessors_.at(pose.x + width_ * pose.y).at(pose.movement);

        int dx = b & 0x07;
        int dy = (b >> 3) & 0x07;
        int dm = (b >> 6) & 0x03;

        dx -= 3;
        dy -= 3;
        dm -= 1;

        return Pose3D{pose.x - dx, pose.y - dy, (pose.movement - dm) % movement_index_count};
    }

private:

    // The width of the map this object holds the total costs for.
    size_t width_;

    // The height of the map this object holds the total costs for.
    size_t height_;

    // For each orientation a single byte is stored that contains
    // the delta values to the predecessor pose.
    using predecessor_array = std::array<uint8_t, movement_index_count>;

    // Stores the delta values for a pose to the predecessor.
    std::vector<predecessor_array> predecessors_;
};

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
        if ((!(p.x < costs.width())) || (!(p.y < costs.height()))) {
            // This pose is outside the map.
            return;
        }

        const Costs::CostType3D cost = costs.get_3d_cost(p.x, p.y, p.movement);
        if (cost == Costs::invalid_3d_cost) {
            // Not a valid pose.
            return;
        }

        // This pose is in the costs object.
        result.emplace_back(std::move(p));
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
    const float angle = costs.angle_index_to_radians(pose.angle_index);

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

float normalized_angle_distance(
    const unsigned int movement_index_a,
    const unsigned int movement_index_b)
{
    // The actual orientation angles in radians.
    const float a = angle_lut[movement_index_a];
    const float b = angle_lut[movement_index_b];

    // The difference between the two orientations, from -pi to pi.
    const float diff = atan2f(sinf(a - b), cosf(a - b));

    // We return the difference between the two orientations, normalized to
    // the range from 0.0 to 1.0.
    return fabsf(diff) / M_PI;
}

float calculate_movemement_cost(
    const Costs& costs,
    const Pose3D& goal_pose,
    const Pose3D& from,
    const Pose3D& to)
{
    // Calculate the cost of a pure forward/backward movement.
    auto pure_movement_cost = [](const Pose3D& from, const Pose3D& to, float cost_value) {
        // Absolute distance driven in the two directions.
        const int dx = abs(static_cast<int>(from.x) - static_cast<int>(to.x));
        const int dy = abs(static_cast<int>(from.y) - static_cast<int>(to.y));

        // Value for driving straight forward/backward.
        float distance = 1.0f;

        if ((dx == 2) || (dy == 2)) {
            // Two forward/backward, one sideward.
            distance = sqrtf(5.0f);
        } else if ((dx == 1) && (dy == 1)) {
            // One forward/backward, one sidward.
            distance = sqrtf(2.0f);
        }
        // else: Pure forward/backward, keep default value.

        // The cost of the neighbour is multiplied by the distance to the cell,
        // so that edges that move diagonally do not have an advantage.
        return distance * (1.0f + cost_value);
    };

    // Calculate the costs of turning on the spot.
    auto pure_turn_cost = [&costs, &goal_pose](const Pose3D& from, const Pose3D& to, float cost_value) {
        // Turning on the spot. We calculate the distance to the goal, and apply
        // a penalty for poses close to the goal. This is so that turns near the
        // goal are discouraged as turning often causes the robot to deviate from
        // the pose and makes it harder to hit the goal exactly.
        const float dx = static_cast<float>(goal_pose.x) - static_cast<float>(to.x);
        const float dy = static_cast<float>(goal_pose.y) - static_cast<float>(to.y);
        const float goal_distance = sqrtf(powf(dx, 2.0f) + powf(dy, 2.0f));

        float goal_penalty = 1.0f + expf(-goal_distance / costs.goal_turn_penalty_distance());
        if (goal_distance < 1.0f) {
            goal_penalty = 2.0f;
        }

        const float angle_distance = normalized_angle_distance(from.movement, to.movement);

        return goal_penalty * (1.0f + angle_factor * angle_distance) * (1.0f + cost_value);
    };

    // The value on the costmap at the 'to' pose.
    const float to_cost = Costs::cost_3d_to_float(
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

            float mc = pure_movement_cost(from, via, to_cost);
            float tc = pure_turn_cost(via, to, to_cost);

            // Calculate the cost of the combined "move + turn" from the costs
            // of separate "pure move" and "pure turn" costs when going over 'via'.
            // The turn cost is half the cost of a pure turn, this way multiple
            // consecutive turns are still "expensive", while only a single turn
            // can be cone relatively cheaply by using such a combined move. Also,
            // this cost value honors the triangle inequality.
            return mc + 0.5f * tc;
        }
    }
}

float calculate_heuristic(
    const DepthHeuristic& heuristic,
    const Pose3D& goal_pose,
    const Pose3D& pose)
{
    float goal_distance = heuristic.get_value(pose.x, pose.y);
    if (std::isnan(goal_distance)) {
        // A fallback if the pose is not in 'heuristic'. In this case there is
        // probably no path to the goal in the search space (otherwise the heuristic
        // map calculation would include the pose), so we could also detect this earlier.
        const float dx = static_cast<float>(goal_pose.x) - static_cast<float>(pose.x);
        const float dy = static_cast<float>(goal_pose.y) - static_cast<float>(pose.y);
        goal_distance = sqrtf(powf(dx, 2.0f) + powf(dy, 2.0f));
    }

    // Calculate the normalized angle differenct between the orientation of the
    // pose and the goal pose.
    const float angle_distance = normalized_angle_distance(pose.movement, goal_pose.movement);

    // Compare to 'pure_movement_cost()': If we drive only forward, without ever
    // turning, far from every obstacle, so that the cost of a cell is 0.0:
    const float movement_cost = goal_distance;

    // Compare to 'pure_turn_cost()', but also to the calculation of the combined
    // "move + turn" step (taking only half the costs of a pure turn). No 'goal_penalty'
    // (see 'pure_turn_cost()' is applied, as we can not know where the turns would occur).
    const float turn_cost = 0.5f * (angle_factor * angle_distance);

    return movement_cost + turn_cost;
}

SearchResult3D plan(
    const std::function<void(const std::string&)>& log_callback,
    const Costs& costs,
    const Pose& start,
    const Pose& goal,
    bool extract_opened_nodes)
{
    // Convert the poses for the trajectory planner to the (more granular
    // in regard to the orientation) poses used here.
    Pose3D start_pose = refine_pose(costs, start);
    Pose3D goal_pose = refine_pose(costs, goal);

    std::ostringstream ss;
    ss
        << "  start pose: " << start_pose;
    log_callback(ss.str());
    ss.str("");
    ss
        << "  goal pose: " << goal_pose;
    log_callback(ss.str());

    // The main heuristic for the 3D search is the distance to the goal, calculated
    // by flood-filling the 3D search space in 2D. While it takes some time to
    // pre-calculate these values before doing the actual search, it pays of on
    // large maps, and does not take much time on small maps.
    DepthHeuristic heuristic(costs, Pose2D{goal_pose.x, goal_pose.y}, true);

    auto goal_reached = [&goal_pose](const Pose3D& pose) {
        return goal_pose == pose;
    };

    auto get_neighbours = [&costs](const Pose3D& pose) {
        return neighbours(costs, pose);
    };

    auto movement_cost = [&costs, &goal_pose](const Pose3D& pose, const Pose3D& neighbour) {
        return calculate_movemement_cost(costs, goal_pose, pose, neighbour);
    };

    auto heuristic_callback = [&costs, &goal_pose, &heuristic](const Pose3D& pose) {
        return calculate_heuristic(heuristic, goal_pose, pose);
    };

    using a_star_type = AStar<
        Pose3D,
        decltype(goal_reached),
        decltype(get_neighbours),
        decltype(movement_cost),
        decltype(heuristic_callback),
        TotalCostsMap,
        PredecessorsMap,
        false>;

    TotalCostsMap total_costs(costs.width(), costs.height());
    PredecessorsMap predecessors(costs.width(), costs.height());

    a_star_type a_star;
    const Path3D path = a_star.search(
        goal_reached,
        get_neighbours,
        movement_cost,
        heuristic_callback,
        start_pose,
        total_costs,
        predecessors);

    SearchResult3D result(std::move(heuristic));
    if (extract_opened_nodes) {
        result.opened_nodes = total_costs.count_opened_nodes();
    }

    result.path = std::move(path);

    for (const Pose3D& pose: result.path) {
        result.path_heuristic.push_back(result.heuristic.get_value(pose.x, pose.y));
    }

    // Create the costs vector in the result. This is done by iterating over
    // pairs in the found path and summing up the cost values along the way.
    result.cost.push_back(0.0f); // Cost of the goal.

    float cost = 0.0f;
    for (size_t i = result.path.size(); i-- > 1;) {
        const Pose3D& node = result.path.at(i);
        const Pose3D& predecessor = result.path.at(i - 1);
        cost += movement_cost(predecessor, node);
        result.cost.push_back(cost);
    }
    std::reverse(result.cost.begin(), result.cost.end());

    return result;
}

std::ostream& operator<<(
    std::ostream& os,
    const three::Pose3D& pose)
{
    os
        << "Pose3D{x = " << pose.x
        << ", y = " << pose.y
        << ", movement = " << pose.movement << "}";

    return os;
}

}
