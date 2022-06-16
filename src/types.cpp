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

}
