#include "trajectory_planner/planning_5d.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/weighted_sum.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <cmath>
#include <optional>
#include <thread>

#include <iostream>
#include <iomanip>

namespace trajectory_planner::five
{

std::vector<Segment> split_path(
    unsigned int angle_granularity,
    const Path& path)
{
    std::vector<Segment> result;

    if (path.size() < 2) {
        return result;
    }

    // Two poses, and the direction when moving from the first to the second.
    using PosePair = std::tuple<Pose, Pose, Direction>;

    std::vector<PosePair> pairs;
    std::transform(
        path.cbegin(),
        path.cend() - 1,
        path.cbegin() + 1,
        std::back_inserter(pairs),
        [angle_granularity](const Pose& from, const Pose& to) {
            Direction d = Direction::TURN;

            // If at least one of the coordinates differ, we did not turn.
            if ((from.x != to.x) || (from.y != to.y)) {
                // The angle when moving from the first pose to the second.
                const double angle = atan2(
                    static_cast<int>(to.y) - static_cast<int>(from.y),
                    static_cast<int>(to.x) - static_cast<int>(from.x));

                // The orientation at the first pose.
                const double orientation = to.angle_index * 2.0 * M_PI / angle_granularity;

                // Determine the direction of the robot when moving from one pose
                // to the other form the difference of the angles.
                const double diff = atan2(sin(orientation - angle), cos(orientation - angle));
                if (abs(diff) < (M_PI / 2.0)) {
                    d = Direction::FORWARD;
                } else {
                    d = Direction::BACKWARD;
                }
            }

            return std::make_tuple(from, to, d);
        });

    // Remove single turns from the path that are between equal movement parts.
    auto del = std::remove_if(
        pairs.begin(),
        pairs.end(),
        [&pairs](const PosePair& p) {
            // The index of the current item.
            size_t index = &p - &(*pairs.begin());
            if ((index == 0) || (index == (pairs.size() - 1))) {
                // Beginning or end of the path.
                return false;
            }

            const Direction pre = std::get<2>(pairs.at(index - 1));
            const Direction d = std::get<2>(p);
            const Direction suc = std::get<2>(pairs.at(index + 1));

            if (d != Direction::TURN) {
                // We only want to return turns.
                return false;
            }

            if ((pre != Direction::TURN) && (pre == suc)) {
                // The direciton of the predecessor and the successor are movements
                // (forward or backward), and they are the same.
                return true;
            }

            return false;
        });
    pairs.erase(del, pairs.end());

    // Split 'pairs' into multiple parts with the same direction.
    auto begin = pairs.cbegin();
    while (begin != pairs.cend()) {
        // The direction of the current segment we are splitting.
        const Direction d = std::get<2>(*begin);

        // A predicate that returns 'true' if a pair has the same direction as the
        // first one in the current range.
        auto same_direction = [d](const PosePair& p) { return std::get<2>(p) == d; };

        // Find the end of the current segment.
        auto last = std::find_if_not(begin, pairs.cend(), same_direction);

        // Copy out the poses of the current segment.
        Path path;
        for (auto it = begin; it < last; it++) {
            if (it == begin) {
                path.push_back(std::get<0>(*it));
            }
            path.push_back(std::get<1>(*it));
        }

        begin = last;

        result.emplace_back(Segment{d, std::move(path)});
    }

    return result;
}

CircleCoordinates circle_coordinates(
    unsigned int pixel_radius)
{
    int canvas_size = 2 * pixel_radius + 1;

    cv::Point center(pixel_radius, pixel_radius);
    cv::Mat canvas = cv::Mat::zeros(canvas_size, canvas_size, CV_8UC1);
    cv::circle(canvas, center, pixel_radius, cv::Scalar(255), cv::FILLED);

    std::vector<std::tuple<int, int>> result;

    for (int y = 0; y < canvas_size; y ++) {
        for (int x = 0; x < canvas_size; x ++) {
            if (canvas.at<unsigned char>(y, x) != 0) {
                const unsigned int dx = x - pixel_radius;
                const unsigned int dy = y - pixel_radius;
                result.emplace_back(std::make_tuple(dx, dy));
            }
        }
    }

    return result;
}

double measure_curvature(
    unsigned int angle_granularity,
    const Path& path,
    size_t start,
    double maximum_distance,
    bool forward)
{
    namespace accumulators = boost::accumulators;

    accumulators::accumulator_set<
        double, accumulators::features<accumulators::tag::weighted_sum>, double> acc;

    double total_distance = 0.0;

    size_t index = start;
    while (total_distance < maximum_distance) {
        // Check if we reached an end.
        if (forward) {
            if (index == (path.size() - 1)) {
                // We reached the end.
                break;
            }
        } else {
            if (index == 0) {
                // We reached the beginning.
                break;
            }
        }

        const Pose& prev = path.at(index);
        index += forward ? 1 : -1;
        const Pose& curr = path.at(index);

        // Calculate the distance between the previous and the current pose.
        const double dx = static_cast<double>(prev.x) - static_cast<double>(curr.x);
        const double dy = static_cast<double>(prev.y) - static_cast<double>(curr.y);
        const double distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
        total_distance += distance;

        // Calculate the change in orientation between the two poses.
        const double prev_orientation = prev.angle_index * 2.0 * M_PI / angle_granularity;
        const double curr_orientation = curr.angle_index * 2.0 * M_PI / angle_granularity;
        const double orientation_diff = atan2(
            sin(prev_orientation - curr_orientation),
            cos(prev_orientation - curr_orientation));

        // The value with which we weight the curvature between the current and the
        // previous pose. Close poses have a higher weight.
        double weight = std::max(0.0, maximum_distance - total_distance);

        acc(abs(orientation_diff), accumulators::weight=weight);
    }

    // Magic value, depending on the maximum length of the path, to scale down
    // the value that is summed up.
    const double denom = maximum_distance * 25.0 * M_PI / 180.0;

    return std::min(1.0, accumulators::weighted_sum(acc) / denom);
}


Costs inflate_path(
    const CircleCoordinatesMap& coordinates_map,
    const unsigned int inflation_lookahead,
    const Costs& costs,
    const Path& path)
{
    // Get the minimum and maximum inflation values. All other values between
    // the extremes are also assumed to be in the map.
    const unsigned int inflation_min = coordinates_map.begin()->first;
    const unsigned int inflation_max = coordinates_map.rbegin()->first;

    std::vector<Pose2D> subset;
    for (const Pose& pose: path) {
        size_t index = &pose - &(*path.begin());

        const double curvature = std::max(
            measure_curvature(costs.angle_granularity(), path, index, inflation_lookahead, true),
            measure_curvature(costs.angle_granularity(), path, index, inflation_lookahead, false));

        unsigned int inflation =
            inflation_min + (inflation_max - inflation_min) * curvature;
        inflation = std::max(inflation_min, inflation);
        inflation = std::min(inflation_max, inflation);
        const CircleCoordinates& coords = coordinates_map.at(inflation);

        const int x = pose.x;
        const int y = pose.y;

        for (const auto& diff: coords) {
            const int new_x = x + std::get<0>(diff);
            const int new_y = y + std::get<1>(diff);

            if ((new_x >= 0) && (new_y >= 0)) {
                subset.emplace_back(
                    Pose2D{
                        static_cast<unsigned int>(new_x),
                        static_cast<unsigned int>(new_y)});
            }
        }
    }

    // Remove duplicates in the generated coordinates;
    std::sort(subset.begin(), subset.end());
    subset.erase(std::unique(subset.begin(), subset.end()), subset.end());

    return costs.intersect(subset);
}

SegmentSearchResult plan_turn_segment(
    const Costs& costs,
    const Segment& segment)
{
    // As we are turning on the spot, only this one pose is the search space.
    std::vector<Pose2D> subset{Pose2D{segment.path.at(0).x, segment.path.at(0).y}};
    Costs search_space = costs.intersect(subset);

    return SegmentSearchResult{segment.path, search_space};
}

SegmentSearchResult plan_movement_segment(
    const CircleCoordinatesMap& coordinates_map,
    const int inflation_lookahead,
    const Costs& costs,
    const Segment& segment)
{
    Costs search_space = inflate_path(
        coordinates_map, inflation_lookahead, costs, segment.path);

    return SegmentSearchResult{segment.path, search_space};
}

SearchResult5D plan(
    const bool multi_threaded,
    unsigned int inflation_radius_pixels,
    unsigned int inflation_lookahead_pixels,
    const Costs& costs,
    const Path& path)
{
    // The width, in pixels, of the maximum inflation around a cell in the 3D path.
    unsigned int maximum_inflation = std::max(2u, inflation_radius_pixels);

    // Prepare the circle coordinates later down used to inflate the 3D path.
    CircleCoordinatesMap coordinates_map;
    for (unsigned int radius = 1; radius <= maximum_inflation; radius++) {
        coordinates_map[radius] = circle_coordinates(radius);
    }

    // Maps the segment number to the result of the search.
    std::unordered_map<size_t, SegmentSearchResult> result_map;
    std::mutex result_map_mutex;

    // Process a single segment and put the result into 'result_map'.
    auto process_segment = [&](size_t index, const Segment& segment) {
        std::optional<SegmentSearchResult> search_result;

        if (segment.direction == Direction::TURN) {
            search_result = plan_turn_segment(costs, segment);
        } else {
            search_result = plan_movement_segment(
                coordinates_map, inflation_lookahead_pixels, costs, segment);
        }

        std::lock_guard<std::mutex> lock(result_map_mutex);
        result_map.emplace(index, std::move(*search_result));
    };

    // We split the 3D path up into multiple segments, each of
    // which is then processed individually in the 5D space.
    std::vector<Segment> segments = split_path(costs.angle_granularity(), path);

    std::unique_ptr<boost::asio::thread_pool> pool;
    if (multi_threaded) {
        pool = std::make_unique<boost::asio::thread_pool>(
            std::thread::hardware_concurrency());
    }

    // Create a plan for each individual segment.
    for (size_t index = 0; index < segments.size(); index++) {
        const Segment& segment = segments.at(index);

        if (multi_threaded) {
            boost::asio::post(
                *pool,
                [index, &segment, &process_segment] { process_segment(index, segment); });
        } else {
            process_segment(index, segment);
        }
    }

    if (multi_threaded) {
        pool->join();
    }

    SearchResult5D result;

    // Sort the individual segment results into a single vector.
    for (size_t index = 0; index < segments.size(); index++) {
        result.segment.emplace_back(result_map.at(index));
    }

    return result;
}

}
