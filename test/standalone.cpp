// Stand alone example on how to use the trajectory planner without ROS.

#include "trajectory_planner/trajectory_planner.hpp"

#include <chrono>
#include <iomanip>
#include <iostream>

int main()
{
    unsigned int angle_granularity = 64;
    double resolution = 0.02;
    double inflation_radius = 2.0;
    size_t map_width = 400;
    size_t map_height = 600;

    auto log_callback = [](const std::string& msg) {
        std::cout << "    " << msg << std::endl;
    };

    trajectory_planner::Polygon footprint;
    footprint.push_back(std::make_tuple(-0.2, 0.2));
    footprint.push_back(std::make_tuple(0.4, 0.2));
    footprint.push_back(std::make_tuple(0.4, -0.2));
    footprint.push_back(std::make_tuple(-0.2, -0.2));

    trajectory_planner::Buffer<double> map(map_width, map_height);

    for (size_t x = 0; x < map.width(); x++) {
        map.at(x, 0) = 1.0;
        map.at(x, map.height() - 1) = 1.0;

        if (x < map.width() / 2) {
            map.at(x, map.height() / 2) = 1.0;
        }
    }
    for (size_t y = 0; y < map.height(); y++) {
        map.at(0, y) = 1.0;
        map.at(map.width() - 1, y) = 1.0;
    }

    std::cout << "creating planner\n";
    auto timestamp_start = std::chrono::steady_clock::now();
    std::unique_ptr<trajectory_planner::TrajectoryPlanner> planner =
        trajectory_planner::TrajectoryPlanner::create_planner(
            log_callback,
            false,
            "",
            angle_granularity,
            resolution,
            inflation_radius,
            footprint,
            map);
    std::chrono::duration<double> creation_duration =
        std::chrono::steady_clock::now() - timestamp_start;

    if (!planner) {
        std::cout << "error creating a planner\n";
        return 1;
    }

    // std::cout << "writing debug maps\n";
    // planner->dump_orientation_maps("/tmp/standalone_orientation_");

    std::cout << "planning\n";
    trajectory_planner::Pose start{50, 50, 0};
    trajectory_planner::Pose goal{50, static_cast<unsigned int>(map.height()) - 50, 0};
    timestamp_start = std::chrono::steady_clock::now();
    trajectory_planner::TrajectoryPlanner::Result result = planner->plan(start, goal);
    std::chrono::duration<double> planning_duration =
        std::chrono::steady_clock::now() - timestamp_start;

    if (result.path.empty()) {
        std::cout << "no path found\n";
    } else {
        std::cout << "path found with " << result.path.size() << " poses\n";
    }

    std::cout
        << std::setprecision(3) << std::fixed
        << "planner creation took " << creation_duration.count() << " sec.\n"
        << "planning took " << planning_duration.count() << " sec.\n";
}
