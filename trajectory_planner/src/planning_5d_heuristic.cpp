#include "trajectory_planner/planning_5d_heuristic.hpp"

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
    : heuristic_(search_space, Pose2D{goal.x, goal.y}, false)
{
}

double DepthHeuristic::value(
    const Pose5D& pose) const
{
    return heuristic_.get_value(pose.x, pose.y);
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
