#include "trajectory_planner/heuristic.hpp"

#include <limits>

namespace trajectory_planner::five
{

double NoneHeuristic::value(
    const Pose5D& /* pose */) const
{
    return 0.0;
}

EuclideanHeuristic::EuclideanHeuristic(
    Pose5D goal)
    : goal_(goal)
{
}

double EuclideanHeuristic::value(
    const Pose5D& pose) const
{
    const double dx = static_cast<double>(goal_.x) - static_cast<double>(pose.x);
    const double dy = static_cast<double>(goal_.y) - static_cast<double>(pose.y);

    return sqrt(pow(dx, 2.0) + pow(dy, 2.0));
}

ManhattanHeuristic::ManhattanHeuristic(
    Pose5D goal)
    : goal_(goal)
{
}

double ManhattanHeuristic::value(
    const Pose5D& pose) const
{
    const double dx = static_cast<double>(goal_.x) - static_cast<double>(pose.x);
    const double dy = static_cast<double>(goal_.y) - static_cast<double>(pose.y);

    return abs(dx) + abs(dy);
}

DepthHeuristic::DepthHeuristic(
    Pose5D goal,
    const Costs& search_space)
{
    // Flood-fill the 5D search space, starting from the goal pose, and use the
    // distance to the goal as the heuristic value.

    values_[Pose2D{goal.x, goal.y}] = 0.0;

    std::unordered_set<Pose2D, boost::hash<Pose2D>> frontier;
    frontier.insert(Pose2D{goal.x, goal.y});

    std::unordered_set<Pose2D, boost::hash<Pose2D>> remaining;
    search_space.export_5d_poses(remaining);
    remaining.erase(Pose2D{goal.x, goal.y});

    while (!remaining.empty()) {
        std::unordered_set<Pose2D, boost::hash<Pose2D>> new_frontier;
        std::unordered_set<Pose2D, boost::hash<Pose2D>> new_remaining;

        for (const Pose2D& r: remaining) {
            std::optional<Pose2D> closest_neighbour;
            double closest_neighbour_distance = std::numeric_limits<double>::max();

            for (const Pose2D& f: frontier) {
                const int rx = static_cast<int>(r.x);
                const int ry = static_cast<int>(r.y);
                const int fx = static_cast<int>(f.x);
                const int fy = static_cast<int>(f.y);

                // Check if the poses are next to each other.
                const bool xn = (rx == (fx - 1)) || (rx == fx) || (rx == (fx + 1));
                const bool yn = (ry == (fy - 1)) || (ry == fy) || (ry == (fy + 1));
                if (!(xn && yn)) {
                    // The pose 'f' is not a neighbour of 'r'.
                    continue;
                }

                // The distance to the neighbour.
                const double dx = rx - fx;
                const double dy = ry - fy;
                const double distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));

                // The "total" heuristic value when going through the neighbour: The
                // value of the neighbour itself, plus the distance from the current
                // pose to the neighbour.
                const double total = values_[f] + distance;

                if (total < closest_neighbour_distance) {
                    // This is the best neighbour we currently know of.
                    closest_neighbour = f;
                    closest_neighbour_distance = total;
                }
            }

            if (closest_neighbour) {
                values_[r] = closest_neighbour_distance;
                new_frontier.insert(r);
            } else {
                new_remaining.insert(r);
            }
        }

        frontier = std::move(new_frontier);
        remaining = std::move(new_remaining);
    }
}

double DepthHeuristic::value(
    const Pose5D& pose) const
{
    return values_.at(Pose2D{pose.x, pose.y});
}

PathHeuristic::PathHeuristic(
    const Path& path,
    const Costs& search_space)
{
    std::vector<Pose2D> path_2d;
    for (const Pose& pose: path) {
        path_2d.emplace_back(Pose2D{pose.x, pose.y});
    }

    std::unordered_set<Pose2D, boost::hash<Pose2D>> poses;
    search_space.export_5d_poses(poses);

    for (const Pose2D& pose: poses) {
        double closest_neighbour_distance = std::numeric_limits<double>::max();

        for (size_t i = 0; i < path.size(); i++) {
            const Pose& path_pose = path.at(i);

            // The distance to the pose on the path.
            const double dx = static_cast<int>(pose.x) - static_cast<int>(path_pose.x);
            const double dy = static_cast<int>(pose.y) - static_cast<int>(path_pose.y);
            const double distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));

            if (distance < closest_neighbour_distance) {
                closest_neighbour_distance = distance;
                values_[pose] = path.size() - 1 - i;
            }
        }
    }
}

double PathHeuristic::value(
    const Pose5D& pose) const
{
    return values_.at(Pose2D{pose.x, pose.y});
}

}
