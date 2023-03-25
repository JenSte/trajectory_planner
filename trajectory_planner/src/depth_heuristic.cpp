#include "trajectory_planner/depth_heuristic.hpp"

#include "trajectory_planner/planning_3d.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <iomanip>
#include <iostream>

namespace
{
    // If this is set to true, during the creation of a 3D heuristic a (huge) number
    // of images is created that shows each step of the flood fill algorithm used
    // to calculate the distance of each pixel to the given goal pose. Check the
    // 'write_debug_image()' function below on how to interpret the images.
    const bool write_3d_debug_image = false;

    // Path prefix for the debug images files.
    const char* debug_image_prefix = "/tmp/debug_3d_heuristic_";
}

namespace trajectory_planner
{

DepthHeuristic::DepthHeuristic(
    const Costs& costs,
    const Pose2D& goal,
    bool costs_3d)
    : width_(costs.width())
    , height_(costs.height())
    , values_(width_ * height_, std::nanf(""))
    , maximum_value_(0.0)
{
    if (write_3d_debug_image && costs_3d) {
        std::cout << "Creating DepthHeuristic for a 3D planner..." << std::endl;
    }

    // The nodes that are between the processed and unprocessed poses.
    std::unordered_set<Pose2D, boost::hash<Pose2D>> frontier;

    // At the start, the values for the goal poses is known.
    set_value(goal.x, goal.y, 0.0);
    frontier.insert(goal);

    // The frontier of the last iteration.
    std::unordered_set<Pose2D, boost::hash<Pose2D>> old_frontier;

    size_t iteration = 0;
    while (!frontier.empty()) {
        iteration++;

        if (write_3d_debug_image && costs_3d) {
            write_debug_image(iteration, frontier, old_frontier);
        }

        // Candidates for the new frontier are all neighbours (direct or diagnoal)
        // of the current frontier set.
        std::vector<Pose2D> candidates;
        for (const Pose2D& p: frontier) {
            // The corner coordinates of a 3x3 region with the frontier pose in the
            // middle, capped to the limits of the map.
            size_t x_min = p.x > 1 ? p.x - 1 : p.x;
            size_t x_max = p.x < width_ - 1 ? p.x + 1 : p.x;
            size_t y_min = p.y > 1 ? p.y - 1 : p.y;
            size_t y_max = p.y < height_ - 1 ? p.y + 1 : p.y;

            // Add all neighbours to the current frontier cells into 'candidates'.
            for (size_t x = x_min; x <= x_max; x++) {
                for (size_t y = y_min; y <= y_max; y++) {
                    candidates.emplace_back(
                        Pose2D{static_cast<unsigned int>(x), static_cast<unsigned int>(y)});
                }
            }
        }

        // Remove duplicates in the candidates vector.
        std::sort(candidates.begin(), candidates.end());
        candidates.erase(std::unique(candidates.begin(), candidates.end()), candidates.end());

        // Add poses from 'candidates' to the result if they are unprocessed.
        std::unordered_set<Pose2D, boost::hash<Pose2D>> new_frontier;
        for (const Pose2D& c: candidates) {
            // Don't look back, if a point is in the current frontier set, it's distance
            // value was just set, and we don't process it in this iteration again.
            if (frontier.find(c) != frontier.end()) {
                continue;
            }

            // Check if there is a cost value for this cell, i.e. if this is a valid
            // pose.
            bool in_costs = false;

            if (costs_3d) {
                for (three::MovementIndex mi = 0; mi < three::movement_index_count; mi++) {
                    if (costs.get_3d_cost(c.x, c.y, mi) != Costs::invalid_3d_cost) {
                        in_costs = true;
                        break;
                    }
                }
            } else {
                for (unsigned int ai = 0; ai < costs.angle_granularity(); ai++) {
                    if (costs.get_5d_cost(c.x, c.y, ai)) {
                        in_costs = true;
                        break;
                    }
                }
            }

            if (!in_costs) {
                // This pose is not in the cost map, so skip it.
                continue;
            }

            // The corner coordinates of a 3x3 region with the candidate in the
            // middle, capped to the limits of the map.
            size_t x_min = c.x > 1 ? c.x - 1 : c.x;
            size_t x_max = c.x < width_ - 1 ? c.x + 1 : c.x;
            size_t y_min = c.y > 1 ? c.y - 1 : c.y;
            size_t y_max = c.y < height_ - 1 ? c.y + 1 : c.y;

            // Search for the shortest path into this cell.
            float smallest_value = std::numeric_limits<float>::max();
            for (size_t x = x_min; x <= x_max; x++) {
                for (size_t y = y_min; y <= y_max; y++) {
                    // We only search in the cells we came from, the ones in the frontier.
                    const auto it = frontier.find(
                        Pose2D{static_cast<unsigned int>(x), static_cast<unsigned int>(y)});
                    if (it == frontier.end()) {
                        // Cell is not in frontier, skip it.
                        continue;
                    }

                    const float neighbour_value = get_value(x, y);
                    if (!std::isnan(neighbour_value)) {
                        // The distance going from the neighbour to the candidate
                        // is either "1.0" when the neighbour is on the same row
                        // or column, or "sqrt(2.0)" for diagonal neighbours.
                        const float dist = ((x == c.x) || (y == c.y)) ? 1.0f : sqrtf(2.0f);

                        // Store the smallest of all neighbouring values.
                        smallest_value = std::min(smallest_value, neighbour_value + dist);
                    }
                }
            }

            // Because there is at least one neighbour (a node in 'frontier' that
            // led us to 'candidate'), this value is not the maximum float value
            // any more.

            // 'smallest_value' calculated above contains the shortest distance to
            // the pose when coming through the current frontier. Check if, on another
            // path, there was already a value set.
            float previous_value = get_value(c.x, c.y);
            if (!std::isnan(previous_value)) {
                if (previous_value < smallest_value) {
                    // There is already a previous value, and it is smaller than
                    // the newly calculated. This means that the current pose was
                    // already reached via some other way and has a shorter distance
                    // to the goal. Do not overwrite it.
                    continue;
                }
            }

            new_frontier.insert(c);
            set_value(c.x, c.y, smallest_value);

            // Also update the maximum value for all poses in the heuristic.
            maximum_value_ = std::max(maximum_value_, smallest_value);
        }

        old_frontier = std::move(frontier);
        frontier = std::move(new_frontier);
    }

    if (write_3d_debug_image && costs_3d) {
        std::cout << "Heuristic creating in " << iteration << " iterations." << std::endl;
    }
}

float DepthHeuristic::get_value(
    unsigned int x,
    unsigned int y) const
{
    if (!(x < width_)) {
        std::ostringstream ss;
        ss
            << "DepthHeuristic::get_value(): x value of " << x
            << " is bigger than the map width of " << width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < height_)) {
        std::ostringstream ss;
        ss
            << "DepthHeuristic::get_value(): y value of " << y
            << " is bigger than the map height of " << height_ << ".";
        throw std::runtime_error(ss.str());
    }

    return values_.at(x + width_ * y);
}

void DepthHeuristic::set_value(
    unsigned int x,
    unsigned int y,
    float value)
{
    if (!(x < width_)) {
        std::ostringstream ss;
        ss
            << "DepthHeuristic::set_value(): x value of " << x
            << " is bigger than the map width of " << width_ << ".";
        throw std::runtime_error(ss.str());
    }

    if (!(y < height_)) {
        std::ostringstream ss;
        ss
            << "DepthHeuristic::set_value(): y value of " << y
            << " is bigger than the map height of " << height_ << ".";
        throw std::runtime_error(ss.str());
    }

    values_.at(x + width_ * y) = value;
}

void DepthHeuristic::write_debug_image(
    size_t iteration,
    const std::unordered_set<Pose2D, boost::hash<Pose2D>>& frontier,
    const std::unordered_set<Pose2D, boost::hash<Pose2D>>& old_frontier) const
{
    cv::Mat canvas = cv::Mat::zeros(height_, width_, CV_8UC1);

    for (size_t y = 0; y < height_; y++) {
        for (size_t x = 0; x < width_; x++) {
            size_t row = height_ - 1 - y;
            size_t column = x;

            const float h = get_value(x, y);
            if (!std::isnan(h)) {
                float grey = h / maximum_value();
                grey = std::max(0.0f, std::min(1.0f, grey));

                canvas.at<unsigned char>(row, column) = 255 * grey;
            }
        }
    }

    // Convert the distance values to a color scale. Unfortunately, we don't have
    // the occupancy map not available here to black out completely unacessable
    // parts, so the pixels with small distance (closer to the goal) are not
    // distinguishable from the occupied space, however this is not really a problem
    // as these images are used to view the progress of the algorithm around the
    // frontier, which is clearly visible.
    cv::Mat color_image;
    cv::applyColorMap(canvas, color_image, cv::COLORMAP_AUTUMN);

    // Mark the current frontier blue.
    for (const Pose2D& p: frontier) {
        size_t row = height_ - 1 - p.y;
        size_t column = p.x;

        color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(255, 0, 0);
    }

    // Mark the old frontier green.
    for (const Pose2D& p: old_frontier) {
        size_t row = height_ - 1 - p.y;
        size_t column = p.x;

        color_image.at<cv::Vec3b>(row, column) = cv::Vec3b(0, 255, 0);
    }

    std::stringstream ss;
    ss
        << debug_image_prefix
        << std::setfill('0') << std::setw(6) << iteration
        << ".png";
    cv::imwrite(ss.str(), color_image);
}

}
