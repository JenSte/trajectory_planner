// Stand alone example on how to use the trajectory planner without ROS.

#include "trajectory_planner/trajectory_planner.hpp"

#include <boost/program_options.hpp>
#include <boost/gil/extension/io/png.hpp>

#include <chrono>
#include <iomanip>
#include <iostream>

boost::program_options::variables_map parse_arguments(int argc, const char *argv[])
{
    namespace po = boost::program_options;

    po::options_description desc("Stand alone trajectory planner demo.");
    desc.add_options()
        // Options that influence how the planner is created.
        ("multi_threaded", po::value<bool>()->default_value(false))
        ("angle_granularity", po::value<unsigned int>()->required())
        ("map", po::value<std::string>()->required())
        ("map_resolution", po::value<double>()->required())
        ("inflation_radius", po::value<double>()->required())

        // Start and goal coordinates are in raw pixel values (y coordinates growing down).
        ("start_x", po::value<unsigned int>()->required())
        ("start_y", po::value<unsigned int>()->required())
        ("start_theta", po::value<double>()->required())
        ("goal_x", po::value<unsigned int>()->required())
        ("goal_y", po::value<unsigned int>()->required())
        ("goal_theta", po::value<double>()->required())

        ("write_debug_images", po::value<bool>()->default_value(true))
        ("debug_files_prefix", po::value<std::string>()->default_value("/tmp/standalone_"))
    ;

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    return vm;
}

int main(int argc, const char* argv[])
{
    boost::program_options::variables_map arguments;
    try {
        arguments = parse_arguments(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "error parsing command line: " << e.what() << std::endl;
        return 1;
    }

    auto log_callback = [](const std::string& msg) {
        std::cout << "    planner: " << msg << std::endl;
    };

    // Footprint of a rectangular robot.
    trajectory_planner::Polygon footprint;
    footprint.push_back(std::make_tuple(-0.21, 0.21));
    footprint.push_back(std::make_tuple(0.41, 0.21));
    footprint.push_back(std::make_tuple(0.41, -0.21));
    footprint.push_back(std::make_tuple(-0.21, -0.21));

    boost::gil::gray8_image_t image;
    std::string filename = arguments["map"].as<std::string>();
    try {
        std::cout
            << "loading map '" << filename << "'...\n";

        boost::gil::read_and_convert_image(filename, image, boost::gil::png_tag());
    } catch (const std::exception& e) {
        std::cerr
            << "error reading map '" << filename << "': " << e.what() << std::endl;
        return 1;
    }

    std::cout << "  map size: " << image.width() << " x " << image.height() << std::endl;
    trajectory_planner::Buffer<double> map(image.width(), image.height());

    auto view = boost::gil::view(image);
    bool map_empty = true;
    for (int y = 0; y < view.height(); ++y) {
        boost::gil::gray8_view_t::x_iterator row = view.row_begin(y);
        for (int x = 0; x < view.width(); ++x) {
            if (row[x] == 0) {
                map.at(x, view.height() - 1 - y) = 1.0;
                map_empty = false;
            }
        }
    }

    if (map_empty) {
        // Warn if there are no obstacles on the input map. This can happen if
        // the input image is not a grayscale image and the conversion fails.
        std::cout << "  warning: map is empty\n";
    }

    std::optional<trajectory_planner::Pose2D> internal_point;

    std::cout << "creating planner...\n";
    auto timestamp_start = std::chrono::steady_clock::now();
    std::unique_ptr<trajectory_planner::TrajectoryPlanner> planner =
        trajectory_planner::TrajectoryPlanner::create_planner(
            log_callback,
            arguments["multi_threaded"].as<bool>(),
            "",
            arguments["angle_granularity"].as<unsigned int>(),
            arguments["map_resolution"].as<double>(),
            arguments["inflation_radius"].as<double>(),
            footprint,
            std::move(map),
            internal_point);
    std::chrono::duration<double> creation_duration =
        std::chrono::steady_clock::now() - timestamp_start;

    if (!planner) {
        std::cout << "error creating a planner\n";
        return 1;
    }

    if (arguments["write_debug_images"].as<bool>()) {
        std::string path = arguments["debug_files_prefix"].as<std::string>();
        std::cout << "writing planner debug images to '" << path << "'...\n";

        planner->write_debug_images(path);
    }

    std::cout << "planning...\n";

    trajectory_planner::TrajectoryPlanner::PlanningParameters pp;
    pp.inflation_5d_radius = 0.2;
    pp.inflation_5d_lookahead = 1.5;
    pp.maximum_wheel_velocity = 1.0;
    pp.maximum_wheel_acceleration = 1.0;
    pp.wheel_distance = 0.3;
    pp.linear_velocity_steps = 5;
    pp.angular_velocity_steps = 5;
    pp.multi_threaded = arguments["multi_threaded"].as<bool>();
    pp.heuristic_type = trajectory_planner::HeuristicType::DEPTH;
    pp.extract_3d_opened_nodes = arguments["write_debug_images"].as<bool>();

    trajectory_planner::Pose start{
        arguments["start_x"].as<unsigned int>(),
        static_cast<unsigned int>(image.height()) - arguments["start_y"].as<unsigned int>(),
        static_cast<unsigned int>(
            arguments["start_theta"].as<double>() * (arguments["angle_granularity"].as<unsigned int>() / 360.0))};
    trajectory_planner::Pose goal{
        arguments["goal_x"].as<unsigned int>(),
        static_cast<unsigned int>(image.height()) - arguments["goal_y"].as<unsigned int>(),
        static_cast<unsigned int>(
            arguments["goal_theta"].as<double>() * (arguments["angle_granularity"].as<unsigned int>() / 360.0))};

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

        if (arguments["write_debug_images"].as<bool>()) {
            std::string path = arguments["debug_files_prefix"].as<std::string>();
            std::cout << "writing result debug images to '" << path << "'...\n";

            planner->write_result_debug_images(result, path);
            planner->write_result_debug_data(result, path);
        }
    } catch (const std::exception& e) {
        std::cerr << "error while planning: " << e.what() << std::endl;
        return 1;
    }
}
