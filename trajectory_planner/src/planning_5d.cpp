#include "trajectory_planner/planning_5d.hpp"
#include "trajectory_planner/a_star.hpp"

#include <boost/accumulators/accumulators.hpp>
#include <boost/accumulators/statistics/weighted_sum.hpp>
#include <boost/asio/post.hpp>
#include <boost/asio/thread_pool.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <optional>
#include <thread>


namespace trajectory_planner::five
{

std::ostream& operator<<(
    std::ostream& os,
    Direction direction)
{
    switch (direction) {
        case Direction::TURN:
            os << "turn";
            break;

        case Direction::FORWARD:
            os << "forward";
            break;

        case Direction::BACKWARD:
            os << "backward";
            break;

        default:
            os.setstate(std::ios_base::failbit);
    }

    return os;
}

// Stores the total costs along a path to a cell during an A* search.
class TotalCostsMap
{
public:

    // Set the total cost value for a pose.
    void set(const Pose5D& pose, float cost)
    {
        total_costs_[pose] = cost;
    }

    // Return the total cost to the pose, or infinity if the pose is not in the object.
    float get(const Pose5D& pose) const
    {
        const auto it = total_costs_.find(pose);
        if (it == total_costs_.end()) {
            return std::numeric_limits<float>::infinity();
        }

        return it->second;
    }

private:

    // Maps the 5D poses to the total costs to this pose.
    std::unordered_map<Pose5D, float, boost::hash<Pose5D>> total_costs_;
};

// Stores the predecessor for a pose in the A* search.
class PredecessorsMap
{
public:

    // Set the predecessor of a pose.
    void set(
        const Pose5D& pose,
        const Pose5D& predecessor)
    {
        predecessors_[pose] = predecessor;
    }

    // Get the predecessor of a pose.
    Pose5D get(
        const Pose5D& pose) const
    {
        return predecessors_.at(pose);
    }

private:

    std::unordered_map<Pose5D, Pose5D, boost::hash<Pose5D>> predecessors_;
};

std::vector<Segment> split_path(
    const three::Path3D& path)
{
    std::vector<Segment> result;

    if (path.size() < 2) {
        return result;
    }

    // Two poses, and the direction when moving from the first to the second.
    using PosePair = std::tuple<three::Pose3D, three::Pose3D, Direction>;

    std::vector<PosePair> pairs;
    std::transform(
        path.cbegin(),
        path.cend() - 1,
        path.cbegin() + 1,
        std::back_inserter(pairs),
        [](const three::Pose3D& from, const three::Pose3D& to) {
            Direction d = Direction::TURN;

            // If at least one of the coordinates differ, we did not turn.
            if ((from.x != to.x) || (from.y != to.y)) {
                // The angle when moving from the first pose to the second.
                const double angle = atan2(
                    static_cast<int>(to.y) - static_cast<int>(from.y),
                    static_cast<int>(to.x) - static_cast<int>(from.x));

                // The orientation at the first pose.
                //const double orientation = to.angle_index * 2.0 * M_PI / angle_granularity;
                const double orientation = three::angle_lut[from.movement];

                // Determine the direction of the robot when moving from one pose
                // to the other form the difference of the angles.
                const double diff = atan2(sin(orientation - angle), cos(orientation - angle));
                if (fabs(diff) < (M_PI / 2.0)) {
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
        three::Path3D path;
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
    const three::Path3D& path,
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

        const three::Pose3D& prev = path.at(index);
        index += forward ? 1 : -1;
        const three::Pose3D& curr = path.at(index);

        // Calculate the distance between the previous and the current pose.
        const double dx = static_cast<double>(prev.x) - static_cast<double>(curr.x);
        const double dy = static_cast<double>(prev.y) - static_cast<double>(curr.y);
        const double distance = sqrt(pow(dx, 2.0) + pow(dy, 2.0));
        total_distance += distance;

        // Calculate the change in orientation between the two poses.
        const double prev_orientation = three::angle_lut[prev.movement];
        const double curr_orientation = three::angle_lut[curr.movement];
        const double orientation_diff = atan2(
            sin(prev_orientation - curr_orientation),
            cos(prev_orientation - curr_orientation));

        // The value with which we weight the curvature between the current and the
        // previous pose. Close poses have a higher weight.
        double weight = std::max(0.0, maximum_distance - total_distance);

        acc(fabs(orientation_diff), accumulators::weight=weight);
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
    const three::Path3D& path)
{
    // Get the minimum and maximum inflation values. All other values between
    // the extremes are also assumed to be in the map.
    const unsigned int inflation_min = coordinates_map.begin()->first;
    const unsigned int inflation_max = coordinates_map.rbegin()->first;

    std::vector<Pose2D> subset;
    for (const three::Pose3D& pose: path) {
        size_t index = &pose - &(*path.begin());

        const double curvature = std::max(
            measure_curvature(path, index, inflation_lookahead, true),
            measure_curvature(path, index, inflation_lookahead, false));

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

    return costs.intersect_5d_costs(subset);
}

std::unique_ptr<Heuristic> create_heuristic(
    HeuristicType heuristic_type,
    const Pose5D& goal,
    const three::Path3D& path,
    const Costs& search_space)
{
    switch (heuristic_type) {
        case HeuristicType::EUCLIDEAN:
            return std::make_unique<EuclideanHeuristic>(goal);

        case HeuristicType::MANHATTAN:
            return std::make_unique<ManhattanHeuristic>(goal);

        case HeuristicType::DEPTH:
            return std::make_unique<DepthHeuristic>(goal, search_space);

        case HeuristicType::PATH:
            return std::make_unique<PathHeuristic>(path, search_space);

        default:
            return std::make_unique<NoneHeuristic>();
    };
}

std::vector<Pose5D> neighbours(
    const double map_resolution,
    const Costs& costs,
    const MotionModel& motion_model,
    bool forward,
    const Pose5D& pose)
{
    // The current position and orientation (in m and rad).
    const double real_x = pose.x * map_resolution;
    const double real_y = pose.y * map_resolution;
    const double real_theta = pose.angle_index * 2 * M_PI / costs.angle_granularity();

    // The real velocities (in m/s and rad/s) of the current pose.
    double real_linear_velocity;
    double real_angular_velocity;
    std::tie(real_linear_velocity, real_angular_velocity) = motion_model.velocities(
        pose.linear_velocity, pose.angular_velocity);

    // "Move" with the current velocities and calculate the new 3D pose.
    double dx, dy, dtheta;
    std::tie(dx, dy, dtheta) = motion_model.calculate_displacement(
        real_theta, real_linear_velocity, real_angular_velocity);

    const double new_x = real_x + dx;
    const double new_y = real_y + dy;
    double new_theta = real_theta + dtheta;
    if (new_theta < 0.0) {
        new_theta += 2 * M_PI;
    }

    if ((new_x < 0.0) || (new_y < 0.0)) {
        // Cell coordinates can only be positive.
        return {};
    }

    const unsigned int x = static_cast<unsigned int>(std::round(new_x / map_resolution));
    const unsigned int y = static_cast<unsigned int>(std::round(new_y / map_resolution));
    unsigned int angle_index =
        static_cast<unsigned int>(std::round(new_theta / ((2 * M_PI) / costs.angle_granularity())));

    angle_index %= costs.angle_granularity();

    if (!costs.get_5d_cost(x, y, angle_index)) {
        // The new pose is not valid.
        return {};
    }

    // Get the possible changed velocities that can be "reached" from the current ones.
    const MotionModel::ValueType& velocities = motion_model.lookup(
        forward, pose.linear_velocity, pose.angular_velocity);

    std::vector<Pose5D> result;
    for (const auto& vels: velocities) {
        Pose5D new_pose{x, y, angle_index, std::get<0>(vels), std::get<1>(vels)};

        if (new_pose == pose) {
            // Skip poses that are the same due to rounding errors.
            continue;
        }

        result.emplace_back(std::move(new_pose));
    }

    return result;
}

std::tuple<double, double, double, double> calculate_movement_distances(
    const double map_resolution,
    const unsigned int angle_granularity,
    const MotionModel& motion_model,
    const Pose5D& pose,
    const Pose5D& neighbour)
{
    // Calculate the direct distance between the two poses on the grid.
    const double cell_dx = static_cast<double>(neighbour.x) - static_cast<double>(pose.x);
    const double cell_dy = static_cast<double>(neighbour.y) - static_cast<double>(pose.y);
    const double cell_distance = sqrt(pow(cell_dx, 2.0) + pow(cell_dy, 2.0));

    // Calculate the "real" coordinates with no rounding, when starting from 'pose'
    // and going with the velocities of said pose.
    const double real_theta = pose.angle_index * 2 * M_PI / angle_granularity;

    double real_linear_velocity;
    double real_angular_velocity;
    std::tie(real_linear_velocity, real_angular_velocity) =
        motion_model.velocities(pose.linear_velocity, pose.angular_velocity);

    double real_dx, real_dy, real_dtheta;
    std::tie(real_dx, real_dy, real_dtheta) =
        motion_model.calculate_displacement(
            real_theta, real_linear_velocity, real_angular_velocity);

    const double real_distance = sqrt(pow(real_dx, 2.0) + pow(real_dy, 2.0));
    const double scaled_real_distance = real_distance / map_resolution;

    const double scaled_real_dx = real_dx / map_resolution;
    const double scaled_real_dy = real_dy / map_resolution;

    const double error_dx = cell_dx - scaled_real_dx;
    const double error_dy = cell_dy - scaled_real_dy;

    const double error = sqrt(pow(error_dx, 2.0) + pow(error_dy, 2.0));

    const double real_angle = atan2(real_dy, real_dx);
    const double cell_angle = atan2(cell_dy, cell_dx);

    const double angel_diff = atan2(sin(real_angle - cell_angle), cos(real_angle - cell_angle));

    return std::make_tuple(cell_distance, scaled_real_distance, error, fabs(angel_diff));
}

SegmentSearchResult plan_turn_segment(
    const Costs& costs,
    const Segment& segment)
{
    // As we are turning on the spot, only this one pose is the search space.
    std::vector<Pose2D> subset{Pose2D{segment.path.at(0).x, segment.path.at(0).y}};
    Costs search_space = costs.intersect_5d_costs(subset);

    Path5D path;
    for (const three::Pose3D& pose: segment.path) {
        const double angle = three::angle_lut[pose.movement];

        path.emplace_back(
            Pose5D{
                pose.x,
                pose.y,
                costs.radians_to_angle_index(angle),
                Pose5D::LinearVelocity(0),
                Pose5D::AngularVelocity(0)});
    }

    return SegmentSearchResult{Direction::TURN, std::move(path), {}, {}, std::move(search_space), nullptr};
}

SegmentSearchResult plan_movement_segment(
    const std::function<void(const std::string&)>& log_callback,
    const size_t index,
    const double map_resolution,
    const CircleCoordinatesMap& coordinates_map,
    const int inflation_lookahead,
    const HeuristicType heuristic_type,
    const Costs& costs,
    const MotionModel& motion_model,
    const Segment& segment)
{
    // Create the space we search by inflating the 3D path.
    Costs search_space = inflate_path(
        coordinates_map, inflation_lookahead, costs, segment.path);

    // The real start pose, in 5D.
    Pose5D real_start{
        segment.path.front().x,
        segment.path.front().y,
        costs.radians_to_angle_index(three::angle_lut[segment.path.front().movement]),
        Pose5D::LinearVelocity(0),
        Pose5D::AngularVelocity(0)};

    // The real goal pose, in 5D.
    Pose5D real_goal{
        segment.path.back().x,
        segment.path.back().y,
        costs.radians_to_angle_index(three::angle_lut[segment.path.back().movement]),
        Pose5D::LinearVelocity(0),
        Pose5D::AngularVelocity(0)};

    Pose5D start = real_start;
    Pose5D goal = real_goal;

    if (segment.direction == Direction::FORWARD) {
        start.linear_velocity = Pose5D::LinearVelocity(1);
        goal.linear_velocity = Pose5D::LinearVelocity(1);
    } else {
        start.linear_velocity = Pose5D::LinearVelocity(-1);
        goal.linear_velocity = Pose5D::LinearVelocity(-1);
    }

    auto goal_reached = [&goal](const Pose5D& pose) {
        return pose == goal;
    };

    auto get_neighbours = [&](const Pose5D& pose) {
         return neighbours(
            map_resolution,
            search_space,
            motion_model,
            segment.direction == Direction::FORWARD,
            pose);
    };

    auto movement_cost = [&](const Pose5D& pose, const Pose5D& neighbour) -> float {

        double cell_distance, real_distance, error, angle_error;
        std::tie(cell_distance, real_distance, error, angle_error) =
            calculate_movement_distances(
                map_resolution,
                search_space.angle_granularity(),
                motion_model,
                pose,
                neighbour);

        // A penalty for lower speeds, to force the search algorithm to prefer
        // higher velocities.
        const unsigned int linear_index = abs(static_cast<int>(pose.linear_velocity));
        const unsigned int indices = motion_model.linear_steps().size();
        const double low_speed_penalty =
            (1.0 / (2.0 * indices)) * (indices - 1 - linear_index);

        return cell_distance + sqrt(error * angle_error) + low_speed_penalty;
    };

    auto timestamp_start = std::chrono::steady_clock::now();
    std::unique_ptr<Heuristic> heuristic =
        create_heuristic(heuristic_type, goal, segment.path, search_space);
    auto timestamp_end = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = timestamp_end - timestamp_start;

    std::ostringstream ss;
    ss
        << "  5D heuristic for segment " << index << " created in "
        << std::setprecision(3) << std::fixed
        << duration.count() << " sec.";
    log_callback(ss.str());

    auto heuristic_callback = [&heuristic](const Pose5D& pose) {
        return heuristic->value(pose);
    };

    using a_star_type = AStar<
        Pose5D,
        decltype(goal_reached),
        decltype(get_neighbours),
        decltype(movement_cost),
        decltype(heuristic_callback),
        TotalCostsMap,
        PredecessorsMap,
        false>;

    TotalCostsMap total_costs;
    PredecessorsMap predecessors;

    a_star_type a_star;
    const Path5D path = a_star.search(
        goal_reached,
        get_neighbours,
        movement_cost,
        heuristic_callback,
        start,
        total_costs,
        predecessors);

    // Create the heuristic values for each pose of the path.
    std::vector<float> path_heuristics;
    for (const Pose5D& pose: path) {
        path_heuristics.push_back(heuristic_callback(pose));
    }

    // Sum up the cost values along the path.
    std::vector<float> path_costs;
    path_costs.push_back(0.0f); // Cost of the goal.

    float cost = 0.0f;
    for (size_t i = path.size(); i-- > 1;) {
        const Pose5D& node = path.at(i);
        const Pose5D& predecessor = path.at(i - 1);
        cost += movement_cost(predecessor, node);
        path_costs.push_back(cost);
    }
    std::reverse(path_costs.begin(), path_costs.end());

    return SegmentSearchResult{
        segment.direction,
        std::move(path),
        std::move(path_heuristics),
        std::move(path_costs),
        std::move(search_space),
        std::move(heuristic)};
}

SearchResult5D plan(
    const std::function<void(const std::string&)>& log_callback,
    bool multi_threaded,
    double map_resolution,
    unsigned int inflation_radius_pixels,
    unsigned int inflation_lookahead_pixels,
    HeuristicType heuristic_type,
    const Costs& costs,
    const MotionModel& motion_model,
    const three::Path3D& path)
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
                log_callback,
                index,
                map_resolution,
                coordinates_map,
                inflation_lookahead_pixels,
                heuristic_type,
                costs,
                motion_model,
                segment);
        }

        std::lock_guard<std::mutex> lock(result_map_mutex);
        result_map.emplace(index, std::move(*search_result));
    };

    // We split the 3D path up into multiple segments, each of
    // which is then processed individually in the 5D space.
    std::vector<Segment> segments = split_path(path);

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
        result.segment.emplace_back(std::move(result_map.at(index)));
    }

    return result;
}

}
