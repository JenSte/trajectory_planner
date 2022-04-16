#include "trajectory_planner/planning_3d.hpp"
#include "trajectory_planner/a_star.hpp"

#include <cmath>
#include <iomanip>
#include <sstream>

namespace trajectory_planner::three
{

AngleIndexLUT create_lookup_table(
    unsigned int angle_granularity)
{
    // Denominator to convert an angle to the corresponding
    // angle index of the trajectory planner.
    const double denom = (2 * M_PI) / angle_granularity;

    // Make sure an angle is between 0 and 2 * pi.
    auto pos = [](const double angle) {
        return (angle < 0.0) ? (angle + 2 * M_PI) : angle;
    };

    return {
        // 1st quadrant.
        static_cast<unsigned int>(std::round(pos(atan2(0, 1)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(1, 2)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(1, 1)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(2, 1)) / denom)),
        // 2nd quadrant.
        static_cast<unsigned int>(std::round(pos(atan2(1, 0)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(2, -1)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(1, -1)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(1, -2)) / denom)),
        // 3rd quadrant.
        static_cast<unsigned int>(std::round(pos(atan2(0, -1)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(-1, -2)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(-1, -1)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(-2, -1)) / denom)),
        // 4th quadrant.
        static_cast<unsigned int>(std::round(pos(atan2(-1, 0)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(-2, 1)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(-1, 1)) / denom)),
        static_cast<unsigned int>(std::round(pos(atan2(-1, 2)) / denom))
    };
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

boost::container::static_vector<Pose3D, 4> neighbours(
    const Costs& costs,
    const AngleIndexLUT& lut,
    const Pose3D& pose)
{
    boost::container::static_vector<Pose3D, 4> result;

    auto add_neighbour = [&result, &costs, &lut](Pose3D p) {
        const unsigned int angle_index = lut.at(p.movement);
        const double cost = costs.get_cost(p.x, p.y, angle_index);
        if (!(cost < (0.5 * Costs::invalid_cost))) {
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

    return result;
}

Pose3D refine_pose(
    const Costs& costs,
    const AngleIndexLUT& lut,
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
    const double cost = costs.get_cost(pose.x, pose.y, lut.at(movement_index));
    if (cost < (0.5 * Costs::invalid_cost)) {
        std::ostringstream ss;
        ss
            << std::setprecision(2)
            << "Refined 3D pose is not valid ("
            << "angle_index = " << pose.angle_index << ", "
            << "angle = " << angle << " rad, "
            << "movement_index = " << movement_index << ", "
            << "rounded angle_index = " << lut.at(movement_index) << ")";
        throw std::runtime_error(ss.str());
    }

    return Pose3D{pose.x, pose.y, movement_index};
}

SearchResult3D plan(
    const Costs& costs,
    const Pose& start,
    const Pose& goal)
{
    AngleIndexLUT lut = create_lookup_table(costs.angle_granularity());

    // Convert the poses for the trajectory planner to the (more granular
    // in regard to the orientation) poses used here.
    Pose3D start_pose = refine_pose(costs, lut, start);
    Pose3D goal_pose = refine_pose(costs, lut, goal);

    auto goal_reached = [&goal_pose](const Pose3D& pose) {
        return goal_pose == pose;
    };

    auto get_neighbours = [&costs, &lut](const Pose3D& pose) {
        return neighbours(costs, lut, pose);
    };

    auto movement_cost = [&costs, &lut, &goal_pose](const Pose3D& pose, const Pose3D& neighbour) {
        const unsigned int angle_index = lut.at(neighbour.movement);
        const double cost = costs.get_cost(neighbour.x, neighbour.y, angle_index);

        if (pose.movement == neighbour.movement) {
            // Linear movement.
            const double dx =
                static_cast<double>(pose.x) - static_cast<double>(neighbour.x);
            const double dy =
                static_cast<double>(pose.y) - static_cast<double>(neighbour.y);
            const double distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));

            // The cost of the neighbour is multiplied by the distance to the cell,
            // so that edges that move diagonally do not have an advantage.
            return distance * (1.0 + cost);
        } else {
            // Turning on the spot. We calculate the distance to the goal, and apply
            // a penalty for poses close to the goal. This is so that turns near the
            // goal are discouraged as turning often causes the robot to deviate from
            // the pose and makes it harder to hit the goal exactly.
            const double dx =
                static_cast<double>(goal_pose.x) - static_cast<double>(neighbour.x);
            const double dy =
                static_cast<double>(goal_pose.y) - static_cast<double>(neighbour.y);
            const double distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));

            double factor = 1.0 + exp(-distance / costs.goal_turn_penalty_distance());
            if (distance < 1.0) {
                factor = 2.0;
            }
            return factor * cost;
        }
    };

    auto heuristic = [&goal_pose](const Pose3D& pose) {
        const double dx =
            static_cast<double>(goal_pose.x) - static_cast<double>(pose.x);
        const double dy =
            static_cast<double>(goal_pose.y) - static_cast<double>(pose.y);
        return sqrt(pow(dx, 2.0) + pow(dy, 2.0));
    };

    using a_star_type = AStar<
        Pose3D,
        decltype(goal_reached),
        decltype(get_neighbours),
        decltype(movement_cost),
        decltype(heuristic)>;

    a_star_type a_star;
    a_star_type::search_result a_star_result = a_star.search(
        goal_reached,
        get_neighbours,
        movement_cost,
        heuristic,
        start_pose);

    SearchResult3D result;

    // Convert the 3D path back to a path for the trajectory planner
    // and create the heuristic vector.
    const Path3D& path = std::get<0>(a_star_result);
    for (const Pose3D& pose: path) {
        result.path.push_back(Pose{pose.x, pose.y, lut[pose.movement]});
        result.heuristic.push_back(heuristic(pose));
    }

    // Create the costs vector in the result. This is done by iterating over
    // pairs in the found path and summing up the cost values along the way.
    if (path.size() >= 2) {
        double cost = 0.0;
        result.cost.push_back(0.0); // Cost of the goal.
        for (size_t i = path.size() - 1; i > 0; --i) {
            const Pose3D& node = path.at(i);
            const Pose3D& predecessor = path.at(i - 1);
            cost += movement_cost(predecessor, node);
            result.cost.push_back(cost);
        }
        std::reverse(result.cost.begin(), result.cost.end());
    }

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
