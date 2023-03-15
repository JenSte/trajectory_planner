#include "trajectory_planner/trajectory_planner.hpp"

#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iomanip>
#include <thread>

namespace
{

// Output an image that shows the cost values used by the 3D planner.
void output_3d_orientation_map(
    const trajectory_planner::Buffer<double>& occupancy_map,
    const trajectory_planner::Costs& costs,
    unsigned int index_3d,
    const std::string& prefix)
{
    size_t width = occupancy_map.width();
    size_t height = occupancy_map.height();

    cv::Mat canvas = cv::Mat::zeros(height, width, CV_8UC1);

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            size_t row = height - 1 - y;
            size_t column = x;

            double cost = costs.get_3d_cost(x, y, index_3d);
            if (cost < (0.5 * trajectory_planner::Costs::invalid_cost_3d)) {
                // Non-occupied pose.
                canvas.at<unsigned char>(row, column) = 0;
            } else {
                // Valid location, create a tone that corresponds to the cost.
                unsigned int grey = 255 * cost;
                grey = std::max(0u, std::min(255u, grey));
                canvas.at<unsigned char>(row, column) = grey;
            }
        }
    }

    // Convert the grayscale values to colors.
    cv::Mat color_image;
    cv::applyColorMap(canvas, color_image, cv::COLORMAP_JET);

    // Do another pass over the colored image and tidy it up a bit.
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            size_t row = height - 1 - y;
            size_t column = x;

            // Color non-occupied cells white.
            double cost = costs.get_3d_cost(x, y, index_3d);
            if (cost < (0.5 * trajectory_planner::Costs::invalid_cost_3d)) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(255, 255, 255);
            }

            // Draw the obstacles in black.
            if (occupancy_map.at(x, y) > 0.5) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    std::stringstream ss;
    ss << prefix << "map_3d_" << std::setw(3) << std::setfill('0') << index_3d << ".png";
    cv::imwrite(ss.str(), color_image);
}

// Output an image that shows where the robot can be placed oriented with a given angle
// index used by the 5D planner.
void output_5d_orientation_map(
    const trajectory_planner::Buffer<double>& occupancy_map,
    const trajectory_planner::Costs& costs,
    unsigned int angle_index,
    const std::string& prefix)
{
    size_t width = occupancy_map.width();
    size_t height = occupancy_map.height();

    // Start with a white image.
    cv::Mat canvas = cv::Mat(height, width, CV_8UC3, cv::Vec3b(255, 255, 255));

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            size_t row = height - 1 - y;
            size_t column = x;

            if (occupancy_map.at(x, y) > 0.5) {
                // Space is occupied in the original floor plan, paint black.
                canvas.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
            } else {
                if (costs.get_5d_cost(x, y, angle_index)) {
                    // Valid location, draw green.
                    canvas.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 255, 0);
                }
            }
        }
    }

    std::stringstream ss;
    ss << prefix << "map_5d_" << std::setw(3) << std::setfill('0') << angle_index << ".png";
    cv::imwrite(ss.str(), canvas);
}

}

namespace trajectory_planner
{

void TrajectoryPlanner::dump_orientation_maps(
    const std::string& prefix) const
{
    boost::asio::thread_pool pool(std::thread::hardware_concurrency());

    // Maps showing the 3D costs.
    for (unsigned int i = 0; i < three::movement_index_count; i++) {
        boost::asio::post(
            pool,
            [i, prefix, this]{
                output_3d_orientation_map(this->original_occupancy_map_, this->costs_, i, prefix);
            }
        );
    }

    // Maps showing the 5D occupancy maps.
    for (unsigned int ai = 0; ai < costs_.angle_granularity(); ai++) {
        boost::asio::post(
            pool,
            [ai, prefix, this]{
                output_5d_orientation_map(this->original_occupancy_map_, this->costs_, ai, prefix);
            }
        );
    }

    pool.join();
}

}
