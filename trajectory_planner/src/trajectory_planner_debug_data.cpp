#include "trajectory_planner/trajectory_planner.hpp"

#include <fstream>
#include <iomanip>
#include <iostream>

namespace
{

void output_result_3d_data(
    const trajectory_planner::three::SearchResult3D& result,
    const std::string& filename)
{
    if (result.path.size() != result.path_heuristic.size()) {
        std::cout
            << "output_result_3d_data(): path_heuristic vector has wrong length." << std::endl;
        return;
    }

    if (result.path.size() != result.cost.size()) {
        std::cout
            << "output_result_3d_data(): cost vector has wrong length." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout
            << "Unable to open '" << filename << "' to write 3D path data." << std::endl;
        return;
    }

    file
        << "x,"
        << "y,"
        << "theta,"
        << "cost,"
        << "heuristic"
        << "\n";

    for (size_t i = 0; i < result.path.size(); i++) {
        const unsigned int x = result.path[i].x;
        const unsigned int y = result.path[i].y;

        file
            << x << ','
            << y << ','
            << trajectory_planner::three::angle_lut[result.path[i].movement] << ','
            << result.cost[i] << ','
            << result.heuristic.get_value(x, y)
            << "\n";
    }
}

void output_result_5d_data(
    const trajectory_planner::five::SegmentSearchResult& segment,
    const trajectory_planner::MotionModel& motion_model,
    const trajectory_planner::Costs& costs,
    const std::string& filename)
{
    if (segment.direction == trajectory_planner::five::Direction::TURN) {
        return;
    }

    if (segment.path.size() != segment.path_heuristics.size()) {
        std::cout
            << "output_result_5d_data(): path_heuristics vector has wrong length." << std::endl;
        return;
    }

    if (segment.path.size() != segment.path_costs.size()) {
        std::cout
            << "output_result_5d_data(): path_costs vector has wrong length." << std::endl;
        return;
    }

    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cout
            << "Unable to open '" << filename << "' to write 5D path data." << std::endl;
        return;
    }

    file
        << "time,"
        << "x,"
        << "y,"
        << "theta,"
        << "linear_velocity_index,"
        << "linear_velocity,"
        << "angular_velocity_index,"
        << "angular_velocity,"
        << "cost,"
        << "heuristic"
        << "\n";

    for (size_t i = 0; i < segment.path.size(); i++) {
        const trajectory_planner::Pose5D::LinearVelocity lvi =
            segment.path[i].linear_velocity;
        const trajectory_planner::Pose5D::AngularVelocity avi =
            segment.path[i].angular_velocity;

        std::tuple<double, double> velocities = motion_model.velocities(lvi, avi);

        file
            << (i * motion_model.time_delta()) << ','
            << segment.path[i].x << ','
            << segment.path[i].y << ','
            << costs.angle_index_to_radians(segment.path[i].angle_index) << ','
            << static_cast<int>(lvi) << ','
            << std::get<0>(velocities) << ','
            << static_cast<int>(avi) << ','
            << std::get<1>(velocities) << ','
            << segment.path_costs[i] << ','
            << segment.path_heuristics[i] << '\n';
    }
}

}

namespace trajectory_planner
{

void TrajectoryPlanner::write_result_debug_data(
    const Result& result,
    const std::string& prefix) const
{
    if (result.search_result_3d.path.empty()) {
        std::cout
            << "write_result_debug_data(): 3D path is empty, not writing any debug data." << std::endl;
        return;
    }

    output_result_3d_data(
        result.search_result_3d,
        prefix + "result_3d_path_data.csv");

    for (size_t i = 0; i < result.search_result_5d.segment.size(); i++) {
        std::ostringstream ss;
        ss
            << prefix
            << "result_5d_"
            << std::setfill('0') << std::setw(3) << i;

        output_result_5d_data(
            result.search_result_5d.segment[i],
            result.motion_model,
            costs_,
            ss.str() + "_path_data.csv");
    }
}

}
