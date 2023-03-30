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

// Put a text on the image.
void write_text(
    cv::Mat& image,
    int column,
    int row,
    const std::string& text)
{
    const cv::Vec3b black(0, 0, 0);
    const cv::Point origin(column, row);

    cv::putText(image, text, origin, cv::FONT_HERSHEY_PLAIN, 1.0, black, 1, cv::LINE_AA);
}

// Add some rows to the given image and add a text to it.
void extend_and_label_image(
    cv::Mat& image,
    std::string label)
{
    cv::Vec3b white(255, 255, 255);

    const int old_height = image.rows;
    image.resize(old_height + 56, white);

    write_text(image, 10, old_height + 24, label);
}

// Output an image that shows the inflated floorplan.
void output_inflated_occupancy_map(
    const trajectory_planner::Buffer<double>& occupancy_map,
    const trajectory_planner::Buffer<double>& cost_map,
    const std::string& prefix)
{
    size_t width = occupancy_map.width();
    size_t height = occupancy_map.height();

    cv::Mat canvas = cv::Mat::zeros(height, width, CV_8UC1);

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            size_t row = height - 1 - y;
            size_t column = x;

            float cost = cost_map.at(x, y);
            cost = std::max(0.0f, std::min(1.0f, cost));

            canvas.at<unsigned char>(row, column) = 255 * cost;
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

            // Draw the obstacles in black.
            if (occupancy_map.at(x, y) > 0.5) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    extend_and_label_image(canvas, "3D inflated occupancy map, black/white");
    extend_and_label_image(color_image, "3D inflated occupancy map");

    // Output a black/white version of this image, makes it easier to inspect
    // the raw pixel values with an image viewer, and a RGB version.
    cv::imwrite(prefix + "bw.png", canvas);
    cv::imwrite(prefix + "color.png", color_image);
}

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

            const trajectory_planner::Costs::CostType3D cost =
                costs.get_3d_cost(x, y, index_3d);
            if (cost == trajectory_planner::Costs::invalid_3d_cost) {
                // Non-occupied pose.
                canvas.at<unsigned char>(row, column) = 0;
            } else {
                // Valid location, create a tone that corresponds to the cost.
                float c = trajectory_planner::Costs::cost_3d_to_float(cost);
                unsigned int grey = 255 * c;
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
            const trajectory_planner::Costs::CostType3D cost =
                costs.get_3d_cost(x, y, index_3d);
            if (cost == trajectory_planner::Costs::invalid_3d_cost) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(255, 255, 255);
            }

            // Draw the obstacles in black.
            if (occupancy_map.at(x, y) > 0.5) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    extend_and_label_image(color_image, "3D costmap");

    const double deg = index_3d * 360.0 / trajectory_planner::three::movement_index_count;
    const double rad = index_3d * 2 * M_PI / trajectory_planner::three::movement_index_count;

    std::stringstream ss;
    ss
        << "Orientation: " << index_3d << "/" << trajectory_planner::three::movement_index_count
        << " (" << std::fixed << std::setprecision(1) << deg
        << " deg, " << std::setprecision(2) << rad << " rad)";
    write_text(color_image, 10, height + 46, ss.str());

    ss.str("");
    ss << prefix << std::setw(3) << std::setfill('0') << index_3d << ".png";
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

    extend_and_label_image(canvas, "5D occupancy map");

    const double deg = angle_index * 360.0 / costs.angle_granularity();
    const double rad = angle_index * 2 * M_PI / costs.angle_granularity();

    std::stringstream ss;
    ss
        << "Orientation: " << angle_index << "/" << costs.angle_granularity()
        << " (" << std::fixed << std::setprecision(1) << deg
        << " deg, " << std::setprecision(2) << rad << " rad)";
    write_text(canvas, 10, height + 46, ss.str());

    ss.str("");
    ss << prefix << "map_5d_" << std::setw(3) << std::setfill('0') << angle_index << ".png";
    cv::imwrite(ss.str(), canvas);
}

void output_depth_heuristic_image(
    const trajectory_planner::DepthHeuristic& heuristic,
    const trajectory_planner::Buffer<double>& occupancy_map,
    double map_resolution,
    const std::string& label,
    const std::string& filename)
{
    size_t width = occupancy_map.width();
    size_t height = occupancy_map.height();

    cv::Mat canvas = cv::Mat::zeros(height, width, CV_8UC1);

    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            size_t row = height - 1 - y;
            size_t column = x;

            const float h = heuristic.get_value(x, y);
            if (!std::isnan(h)) {
                float grey = h / heuristic.maximum_value();
                grey = std::max(0.0f, std::min(1.0f, grey));

                canvas.at<unsigned char>(row, column) = 255 * grey;
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

            // Cells where no heuristic value exists in white.
            const float h = heuristic.get_value(x, y);
            if (std::isnan(h)) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(255, 255, 255);
            }

            // Draw the obstacles in black.
            if (occupancy_map.at(x, y) > 0.5) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    extend_and_label_image(color_image, label);

    std::stringstream ss;
    ss
        << "Max. distance to goal: "
        << std::fixed << std::setprecision(2) << heuristic.maximum_value()
        << " px / " << heuristic.maximum_value() * map_resolution << " m";
    write_text(color_image, 10, height + 46, ss.str());

    cv::imwrite(filename, color_image);
}

void output_result_3d_heuristic_image(
    const trajectory_planner::DepthHeuristic& heuristic,
    const trajectory_planner::Buffer<double>& occupancy_map,
    double map_resolution,
    const std::string& filename)
{
    output_depth_heuristic_image(
        heuristic, occupancy_map, map_resolution, "3D heuristic", filename);
}

void output_result_3d_opened_nodes_image(
    const trajectory_planner::three::SearchResult3D::OpenedNodesMap& opened_nodes,
    const trajectory_planner::Buffer<double>& occupancy_map,
    const std::string& filename)
{
    const size_t width = occupancy_map.width();
    const size_t height = occupancy_map.height();

    cv::Mat canvas = cv::Mat::zeros(height, width, CV_8UC1);

    // Return the grayscale value for a given number of node visits.
    auto color = [](unsigned int visited) -> unsigned char {
        double grey = visited;
        grey /= trajectory_planner::three::movement_index_count;
        grey = std::max(0.0, std::min(1.0, grey));

        return 100 + 155 * grey;
    };

    size_t total_opened = 0;

    for (const auto it: opened_nodes) {
        const size_t row = height - 1 - it.first.y;
        const size_t column = it.first.x;

        total_opened += it.second;
        canvas.at<unsigned char>(row, column) = color(it.second);
    }

    // Convert the grayscale values to colors.
    cv::Mat color_image;
    cv::applyColorMap(canvas, color_image, cv::COLORMAP_JET);

    // Do another pass over the colored image and tidy it up a bit.
    for (unsigned int y = 0; y < height; y++) {
        for (unsigned int x = 0; x < width; x++) {
            const size_t row = height - 1 - y;
            const size_t column = x;

            // Cells that were not opend in white.
            const auto it = opened_nodes.find(trajectory_planner::Pose2D{x, y});
            if (it == opened_nodes.end()) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(255, 255, 255);
            }

            // Draw the obstacles in black.
            if (occupancy_map.at(x, y) > 0.5) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    extend_and_label_image(color_image, "3D opened nodes");

    for (unsigned int c = 1; c <= 16; c++) {
        cv::Mat m(10, 10, CV_8UC1, color(c));
        cv::Mat n;
        cv::applyColorMap(m, n, cv::COLORMAP_JET);

        cv::Rect rect(width - 20 - (160 - c * n.cols), height + 10, n.cols, n.rows);
        cv::Mat roi(color_image, rect);
        n.copyTo(roi);
    }

    std::stringstream ss;
    ss
        << "Total opened nodes: " << total_opened;
    write_text(color_image, 10, height + 46, ss.str());

    cv::imwrite(filename, color_image);
}

void output_result_3d_path_image(
    const trajectory_planner::three::Path3D& path,
    const trajectory_planner::Buffer<double>& occupancy_map,
    const std::string& filename)
{
    size_t width = occupancy_map.width();
    size_t height = occupancy_map.height();

    cv::Mat color_image(height, width, CV_8UC3, cv::Vec3b(255, 255, 255));

    for (const trajectory_planner::three::Pose3D& pose: path) {
        size_t row = height - 1 - pose.y;
        size_t column = pose.x;

        color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(3, 252, 44);
    }

    // Do another pass over the colored image and tidy it up a bit.
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            size_t row = height - 1 - y;
            size_t column = x;

            // Draw the obstacles in black.
            if (occupancy_map.at(x, y) > 0.5) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    extend_and_label_image(color_image, "3D path");

    cv::imwrite(filename, color_image);
}

void output_result_5d_path_image(
    const trajectory_planner::five::SegmentSearchResult& segment,
    const trajectory_planner::Buffer<double>& occupancy_map,
    const std::string& filename)
{
    size_t width = occupancy_map.width();
    size_t height = occupancy_map.height();

    cv::Mat color_image(height, width, CV_8UC3, cv::Vec3b(255, 255, 255));

    // Draw the inflated 3D path.
    std::unordered_set<trajectory_planner::Pose2D, boost::hash<trajectory_planner::Pose2D>> cost_poses;
    segment.costs.export_5d_poses(cost_poses);
    for (const trajectory_planner::Pose2D& pose: cost_poses) {
        size_t row = height - 1 - pose.y;
        size_t column = pose.x;

        color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 102, 255);
    }

    // Draw the 5D path on top.
    for (const trajectory_planner::Pose5D& pose: segment.path) {
        size_t row = height - 1 - pose.y;
        size_t column = pose.x;

        color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(3, 252, 44);
    }

    // Do another pass over the colored image and tidy it up a bit.
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            size_t row = height - 1 - y;
            size_t column = x;

            // Draw the obstacles in black.
            if (occupancy_map.at(x, y) > 0.5) {
                color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 0, 0);
            }
        }
    }

    extend_and_label_image(color_image, "5D path (on inflated 3D path)");

    std::stringstream ss;
    ss
        << "Direction: " << segment.direction
        << ", start: " << segment.path.front()
        << ", goal: " << segment.path.back();
    write_text(color_image, 10, height + 46, ss.str());

    cv::imwrite(filename, color_image);
}

void output_result_5d_heuristic_image(
    const trajectory_planner::five::SegmentSearchResult& segment,
    const trajectory_planner::Buffer<double>& occupancy_map,
    double map_resolution,
    const std::string& filename)
{
    trajectory_planner::five::DepthHeuristic* dh =
        dynamic_cast<trajectory_planner::five::DepthHeuristic*>(segment.heuristic.get());
    if (nullptr == dh) {
        // This function only supports the case where the used 5D heuristic is
        // a depth heuristic.
        return;
    }

    output_depth_heuristic_image(
        dh->internal_heuristic(), occupancy_map, map_resolution, "5D heuristic", filename);
}

}

namespace trajectory_planner
{

void TrajectoryPlanner::write_debug_images(
    const std::string& prefix) const
{
    boost::asio::thread_pool pool(std::thread::hardware_concurrency());

    // Map showing the inflated obstacles.
    boost::asio::post(
        pool,
        [prefix, this]{
            output_inflated_occupancy_map(
                this->original_occupancy_map_,
                this->cost_map_,
                prefix + "3d_inflated_occupancy_");
        });

    // Maps showing the 3D costs.
    for (unsigned int i = 0; i < three::movement_index_count; i++) {
        boost::asio::post(
            pool,
            [i, prefix, this]{
                output_3d_orientation_map(
                    this->original_occupancy_map_,
                    this->costs_,
                    i,
                    prefix + "3d_orientation_");
            }
        );
    }

    // Maps showing the 5D occupancy maps.
    for (unsigned int ai = 0; ai < costs_.angle_granularity(); ai++) {
        boost::asio::post(
            pool,
            [ai, prefix, this]{
                output_5d_orientation_map(
                    this->original_occupancy_map_,
                    this->costs_,
                    ai,
                    prefix + "5d_orientation_");
            }
        );
    }

    pool.join();
}

void TrajectoryPlanner::write_result_debug_images(
    const Result& result,
    const std::string& prefix) const
{
    // Show the 3D heuristic.
    output_result_3d_heuristic_image(
        result.search_result_3d.heuristic,
        original_occupancy_map_,
        map_resolution_,
        prefix + "3d_heuristic.png");

    // Show the nodes (only the X/Y coordinates, not the orientation, obviously) that
    // were looked at during the 3D search
    output_result_3d_opened_nodes_image(
        result.search_result_3d.opened_nodes,
        original_occupancy_map_,
        prefix + "3d_opened_nodes.png");

    // Mark the 3D path on the image.
    output_result_3d_path_image(
        result.search_result_3d.path,
        original_occupancy_map_,
        prefix + "3d_path.png");

    for (size_t i = 0; i < result.search_result_5d.segment.size(); i++) {
        std::ostringstream ss;
        ss
            << prefix
            << "5d_"
            << std::setfill('0') << std::setw(3) << i;

        // Show the inflated 3D path and the heuristic values.
        output_result_5d_heuristic_image(
            result.search_result_5d.segment[i],
            original_occupancy_map_,
            map_resolution_,
            ss.str() + "_heuristic.png");

        // Show the 5D path on the image, on top of the inflated 3D path.
        output_result_5d_path_image(
            result.search_result_5d.segment[i],
            original_occupancy_map_,
            ss.str() + "_path.png");
    }
}

}
