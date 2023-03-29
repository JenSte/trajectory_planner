#include "trajectory_planner/planning_5d_heuristic.hpp"

#include <limits>

namespace trajectory_planner::five
{

float NoneHeuristic::value(
    const Pose5D& /* pose */) const
{
    return 0.0f;
}

EuclideanHeuristic::EuclideanHeuristic(
    Pose5D goal)
    : goal_(goal)
{
}

float EuclideanHeuristic::value(
    const Pose5D& pose) const
{
    const float dx = static_cast<float>(goal_.x) - static_cast<float>(pose.x);
    const float dy = static_cast<float>(goal_.y) - static_cast<float>(pose.y);

    return sqrt(pow(dx, 2.0f) + pow(dy, 2.0f));
}

ManhattanHeuristic::ManhattanHeuristic(
    Pose5D goal)
    : goal_(goal)
{
}

float ManhattanHeuristic::value(
    const Pose5D& pose) const
{
    const float dx = static_cast<float>(goal_.x) - static_cast<float>(pose.x);
    const float dy = static_cast<float>(goal_.y) - static_cast<float>(pose.y);

    return abs(dx) + abs(dy);
}

DepthHeuristic::DepthHeuristic(
    Pose5D goal,
    const Costs& search_space)
    : heuristic_(search_space, Pose2D{goal.x, goal.y}, false)
{
}

float DepthHeuristic::value(
    const Pose5D& pose) const
{
    return heuristic_.get_value(pose.x, pose.y);
}

PathHeuristic::PathHeuristic(
    const three::Path3D& path,
    const Costs& search_space)
{
    std::vector<Pose2D> path_2d;
    for (const three::Pose3D& pose: path) {
        path_2d.emplace_back(Pose2D{pose.x, pose.y});
    }

    std::unordered_set<Pose2D, boost::hash<Pose2D>> poses;
    search_space.export_5d_poses(poses);

    for (const Pose2D& pose: poses) {
        float closest_neighbour_distance = std::numeric_limits<float>::max();

        for (size_t i = 0; i < path.size(); i++) {
            const three::Pose3D& path_pose = path.at(i);

            // The distance to the pose on the path.
            const float dx = static_cast<int>(pose.x) - static_cast<int>(path_pose.x);
            const float dy = static_cast<int>(pose.y) - static_cast<int>(path_pose.y);
            const float distance = sqrt(pow(dx, 2.0f) + pow(dy, 2.0f));

            if (distance < closest_neighbour_distance) {
                closest_neighbour_distance = distance;
                values_[pose] = path.size() - 1 - i;
            }
        }
    }
}

float PathHeuristic::value(
    const Pose5D& pose) const
{
    return values_.at(Pose2D{pose.x, pose.y});
}

}
