// Stand alone example on how to use the trajectory planner without ROS.

#include "trajectory_planner/trajectory_planner.hpp"

#include <boost/gil/extension/io/png.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>

// The image file to load as the map. Path is relative to the current director.
#define MAP_FILE "../map/test.png"

// The start pose.
#define START_X_METER 1.5
#define START_Y_METER 1.5
#define START_THETA_DEGREE 0

// The goal pose.
#define GOAL_X_METER 1.5
#define GOAL_Y_METER 4.5
#define GOAL_THETA_DEGREE 180

// Whether or not to write debug images out to '/tmp/'.
#define WRITE_DEBUG_IMAGES false

int main()
{
    unsigned int angle_granularity = 128;
    double resolution = 0.02;
    double inflation_radius = 1.0;

    auto log_callback = [](const std::string& msg) {
        std::cout << "    " << msg << std::endl;
    };

    trajectory_planner::Polygon footprint;
    footprint.push_back(std::make_tuple(-0.2, 0.2));
    footprint.push_back(std::make_tuple(0.4, 0.2));
    footprint.push_back(std::make_tuple(0.4, -0.2));
    footprint.push_back(std::make_tuple(-0.2, -0.2));

    boost::gil::gray8_image_t image;
    try {
        boost::gil::read_and_convert_image(MAP_FILE, image, boost::gil::png_tag());
    } catch (const std::exception& e) {
        std::cerr << "error reading map '" MAP_FILE "': " << e.what() << std::endl;
        return 1;
    }

    std::cout << "map size: " << image.width() << " x " << image.height() << std::endl;
    trajectory_planner::Buffer<double> map(image.width(), image.height());

    auto view = boost::gil::view(image);
    for (int y = 0; y < view.height(); ++y) {
        boost::gil::gray8_view_t::x_iterator row = view.row_begin(y);
        for (int x = 0; x < view.width(); ++x) {
            if (row[x] == 0) {
                map.at(x, view.height() - 1 - y) = 1.0;
            }
        }
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
            std::move(map));
    std::chrono::duration<double> creation_duration =
        std::chrono::steady_clock::now() - timestamp_start;

    if (!planner) {
        std::cout << "error creating a planner\n";
        return 1;
    }

    if (WRITE_DEBUG_IMAGES) {
        std::cout << "writing debug maps\n";
        planner->dump_orientation_maps("/tmp/standalone_orientation_");
    }

    std::cout << "planning\n";

    trajectory_planner::TrajectoryPlanner::PlanningParameters pp;
    pp.inflation_5d_radius = 0.2;
    pp.inflation_5d_lookahead = 0.5;
    pp.maximum_wheel_velocity = 1.0;
    pp.maximum_wheel_acceleration = 1.0;
    pp.wheel_distance = 0.3;
    pp.linear_velocity_steps = 5;
    pp.angular_velocity_steps = 5;
    pp.multi_threaded = false;
    pp.heuristic_type = trajectory_planner::HeuristicType::DEPTH;

    trajectory_planner::Pose start{
        static_cast<unsigned int>(START_X_METER / resolution),
        static_cast<unsigned int>(START_Y_METER / resolution),
        static_cast<unsigned int>(START_THETA_DEGREE * (angle_granularity / 360.0))};
    trajectory_planner::Pose goal{
        static_cast<unsigned int>(GOAL_X_METER / resolution),
        static_cast<unsigned int>(GOAL_Y_METER / resolution),
        static_cast<unsigned int>(GOAL_THETA_DEGREE * (angle_granularity / 360.0))};

    std::cout << "start: " << start << std::endl;
    std::cout << "goal:  " << goal << std::endl;

    try {
        timestamp_start = std::chrono::steady_clock::now();
        trajectory_planner::TrajectoryPlanner::Result result = planner->plan(pp, start, goal);
        std::chrono::duration<double> planning_duration =
            std::chrono::steady_clock::now() - timestamp_start;

        if (result.search_result_3d.path.empty()) {
            std::cout << "no 3D path found\n";
        } else {
            std::cout << "3D path found with " << result.search_result_3d.path.size() << " poses\n";

            for (const trajectory_planner::five::SegmentSearchResult& seg: result.search_result_5d.segment) {
                std::cout << "5D segment found with " << seg.path.size() << " poses\n";
            }
        }

        std::cout
            << std::setprecision(3) << std::fixed
            << "planner creation took " << creation_duration.count() << " sec.\n"
            << "planning took " << planning_duration.count() << " sec.\n";

        if (WRITE_DEBUG_IMAGES) {
            // Mark the 3D path on the image.

            auto view = boost::gil::view(image);
            for (const trajectory_planner::Pose& pose: result.search_result_3d.path) {
                boost::gil::gray8_view_t::x_iterator row =
                    view.row_begin(view.height() - 1 - pose.y);
                row[pose.x] = 128;
            }

            boost::gil::write_view("/tmp/standalone_3d_path.png", view, boost::gil::png_tag());
        }
    } catch (const std::exception& e) {
        std::cerr << "error while planning: " << e.what() << std::endl;
        return 1;
    }
}
