#include "trajectory_planner/types.hpp"

namespace trajectory_planner
{

std::ostream& operator<<(
    std::ostream& os,
    HeuristicType ht)
{
    switch (ht) {
        case HeuristicType::NONE:
            os << "none";
            break;

        case HeuristicType::EUCLIDEAN:
            os << "euclidean";
            break;

        case HeuristicType::MANHATTAN:
            os << "manhattan";
            break;

        case HeuristicType::DEPTH:
            os << "depth";
            break;

        case HeuristicType::PATH:
            os << "path";
            break;

        default:
            os.setstate(std::ios_base::failbit);
    }

    return os;
}

std::ostream& operator<<(
    std::ostream& os,
    const Pose& pose)
{
    os
        << "Pose{x = " << pose.x
        << ", y = " << pose.y
        << ", angle_index = " << pose.angle_index << "}";

    return os;
}

std::ostream& operator<<(
    std::ostream& os,
    const Pose2D& pose)
{
    os
        << "Pose2D{x = " << pose.x
        << ", y = " << pose.y << "}";

    return os;
}

std::ostream& operator<<(
    std::ostream& os,
    const Pose5D& pose)
{
    os
        << "Pose5D{x = " << pose.x
        << ", y = " << pose.y
        << ", angle = " << pose.angle_index
        << ", lin. vel. = " << static_cast<int>(pose.linear_velocity)
        << ", ang. vel. = " << static_cast<int>(pose.angular_velocity) << "}";

    return os;
}

}
