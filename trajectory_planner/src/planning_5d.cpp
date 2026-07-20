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

std::vector<Segment> split_movement_segment(
    const std::function<void(const std::string&)>& log_callback,
    double map_resolution,
    const Costs& costs,
    const MotionModel& motion_model,
    const Segment& segment)
{
    // The start and end indices (of poses in 'segment.path') of
    // runs with consecutive poses that have the same direction.
    using Run = std::tuple<size_t, size_t>;

    // The runs with consecutive orientations found in 'segment.path'.
    std::vector<Run> all_runs;

    {
        // Find consecutive poses with identical orientation values.

        std::optional<three::MovementIndex> last_movement;
        size_t start_index;

        for (size_t i = 0; i < segment.path.size(); i++) {
            if (!last_movement) {
                // 'last_movement' has not been set yet. We are at the beginning
                // of 'segment.path', start the first run.
                last_movement = segment.path.at(i).movement;
                start_index = i;
            } else {
                if (*last_movement != segment.path.at(i).movement) {
                    // The movement has changed, we've finished a run.
                    size_t end_index = i - 1;

                    all_runs.push_back(std::make_tuple(start_index, end_index));

                    // Start the next run.
                    last_movement = segment.path.at(i).movement;
                    start_index = i;
                }
            }
        }

        // Remove a straight part found at the start of 'segment.path', it makes
        // no sense of splitting this. The same thing is not needed at the end
        // of 'all_runs', because of how the loop above terminates these runs at
        // the end are not generated.
        if (!all_runs.empty()) {
            size_t first_start = std::get<0>(all_runs.at(0));

            if (first_start == 0) {
                all_runs.erase(all_runs.begin());
            }
        }
    }

    // The valid runs: The ones that are long enough.
    std::vector<Run> valid_runs;

    {
        // We consider a run long enough if it is possible for the vehicle to
        // accelerate from 0 to the maximum speed and then brake down again within
        // the length of the run.
        double minimum_distance_meters = 0.0;
        for (double velocity: motion_model.linear_steps()) {
            minimum_distance_meters += velocity * motion_model.time_delta();
        }
        minimum_distance_meters *= 2.5;

        // The 3D path poses are all pixel coordinates, so the minimum length
        // has to be converted to this domain.
        const double minimum_distance_pixels =
            minimum_distance_meters / map_resolution;

        std::ostringstream ss;
        ss
            << "  Minimum length for splitting straight segments: "
            << std::fixed << std::setprecision(2)
            << minimum_distance_pixels << " pixels.";
        log_callback(ss.str());

        for (const Run& run: all_runs) {
            // The start and end poses of the run.
            three::Pose3D start = segment.path.at(std::get<0>(run));
            three::Pose3D end = segment.path.at(std::get<1>(run));

            // Calculate the length of this run.
            const double dx = static_cast<double>(start.x) - static_cast<double>(end.x);
            const double dy = static_cast<double>(start.y) - static_cast<double>(end.y);
            const double dist = sqrt(pow(dx, 2.0) + pow(dy, 2.0));

            if (dist > minimum_distance_pixels) {
                valid_runs.push_back(run);
            }
        }
    }

    // These are the poses (indices of the poses) in the middle of the valid runs
    // that are used to split up the original 3D path.
    std::vector<size_t> split_points;
    {
        // Start of the path.
        split_points.push_back(0);

        // The pose (or, the index of the pose) in the middle of a valid run.
        for (const Run& run: valid_runs) {
            const size_t s = std::get<0>(run);
            const size_t e = std::get<1>(run);

            split_points.push_back(s + (e - s) / 2);
        }

        // End of the path.
        split_points.push_back(segment.path.size() - 1);
    }

    std::vector<Segment> result;

    {
        // Take pairs of the split points and create new segments that
        // contain the poses between the two split points.

        for (size_t i = 0; i < split_points.size() - 1; i++) {
            const size_t s = split_points.at(i);
            const size_t e = split_points.at(i + 1);

            Segment new_segment;
            new_segment.direction = segment.direction;
            std::copy(
                segment.path.begin() + s,
                segment.path.begin() + e,
                std::back_inserter(new_segment.path));

            // The first and the last pose of the split up (original) movement
            // segment have a small speed value, while the start/goal poses
            // of the newly generated segments inbetween have a high speed
            // value. No need to stop at the points where the straight parts
            // where split up.
            const bool start_high_speed = i != 0;
            const bool goal_high_speed = (i + 1) != (split_points.size() - 1);

            new_segment.start_pose =
                convert_pose(
                    costs,
                    motion_model,
                    new_segment.path.front(),
                    new_segment.direction,
                    start_high_speed);
            new_segment.goal_pose =
                convert_pose(
                    costs,
                    motion_model,
                    new_segment.path.back(),
                    new_segment.direction,
                    goal_high_speed);

            result.emplace_back(std::move(new_segment));
        }
    }

    return result;
}

std::vector<Segment> split_path(
    const std::function<void(const std::string&)>& log_callback,
    double map_resolution,
    const Costs& costs,
    const MotionModel& motion_model,
    bool split_long_5d_segments,
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

        Segment new_segment;
        new_segment.direction = d;

        // Copy out the poses of the current segment.
        for (auto it = begin; it < last; it++) {
            if (it == begin) {
                new_segment.path.push_back(std::get<0>(*it));
            }
            new_segment.path.push_back(std::get<1>(*it));
        }
        begin = last;

        new_segment.start_pose = convert_pose(
            costs,
            motion_model,
            new_segment.path.front(),
            new_segment.direction,
            false);
        new_segment.goal_pose = convert_pose(
            costs,
            motion_model,
            new_segment.path.back(),
            new_segment.direction,
            false);

        result.emplace_back(std::move(new_segment));
    }

    if (!split_long_5d_segments) {
        log_callback("Splitting of long straight segments for 5D search is disabled.");
        return result;
    }

    // The elements in 'result', but with another split-procedure applied that
    // tries to split up long forward/backward movement segments even further.
    std::vector<Segment> split_result;

    for (const Segment& segment: result) {
        if (segment.direction == Direction::TURN) {
            split_result.push_back(segment);
        } else {
            // Split movement segment further up.
            std::vector<Segment> segments = split_movement_segment(
                log_callback,
                map_resolution,
                costs,
                motion_model,
                segment);

            split_result.insert(
                split_result.end(),
                segments.begin(),
                segments.end());
        }
    }

    return split_result;
}

void print_segments(
    const std::function<void(const std::string&)>& log_callback,
    const std::vector<Segment>& segments)
{
    std::ostringstream ss;
    ss << "  3D path was split into " << segments.size() << " segment(s):";
    log_callback(ss.str());

    for (size_t i = 0; i < segments.size(); i++) {
        const Segment& segment = segments.at(i);

        ss.str("");
        ss << "    segment #" << i << ":";
        log_callback(ss.str());

        ss.str("");
        ss << "      direction: " << segment.direction;
        log_callback(ss.str());

        ss.str("");
        ss << "      poses: " << segment.path.size();
        log_callback(ss.str());

        ss.str("");
        ss << "      start pose: " << segment.path.front();
        log_callback(ss.str());

        ss.str("");
        ss << "                  " << segment.start_pose;
        log_callback(ss.str());

        ss.str("");
        ss << "      goal pose:  " << segment.path.back();
        log_callback(ss.str());

        ss.str("");
        ss << "                  " << segment.goal_pose;
        log_callback(ss.str());
    }
}

Pose5D convert_pose(
    const Costs& costs,
    const MotionModel& motion_model,
    const three::Pose3D& pose,
    Direction direction,
    bool high_speed)
{
    Pose5D p{
        pose.x,
        pose.y,
        costs.radians_to_angle_index(three::angle_lut[pose.movement]),
        Pose5D::LinearVelocity(0),
        Pose5D::AngularVelocity(0)};

    if (direction == Direction::FORWARD) {
        if (high_speed) {
            p.linear_velocity = Pose5D::LinearVelocity(motion_model.linear_steps().size() - 1);
        } else {
            p.linear_velocity = Pose5D::LinearVelocity(1);
        }
    } else if (direction == Direction::BACKWARD) {
        if (high_speed) {
            p.linear_velocity = Pose5D::LinearVelocity(-(motion_model.linear_steps().size() - 1));
        } else {
            p.linear_velocity = Pose5D::LinearVelocity(-1);
        }
    }

    return p;
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

    const unsigned int inflation_end_1 = std::min(inflation_min + 2, inflation_max);
    const unsigned int inflation_end_2 = std::min(inflation_min + 1, inflation_max);

    const size_t path_length = path.size();

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

        if ((index < 2) || (index >= path_length - 2)) {
            inflation = std::max(inflation, inflation_end_1);
        } else if ((index < 4) || (index >= path_length - 4)) {
            inflation = std::max(inflation, inflation_end_2);
        }

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

    // The real start and goal poses, in 5D.
    Pose5D start = segment.start_pose;
    Pose5D goal = segment.goal_pose;

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

        unsigned turn = abs(
            static_cast<int>(pose.linear_velocity) - static_cast<int>(neighbour.linear_velocity));
        double turn_penalty = turn / 10.0;

        return cell_distance
                * (1.0 + low_speed_penalty)
                * (1.0 + turn_penalty);
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
        // The heuristic is created by flood-filling starting from the goal pose,
        // with a lookahead of 2 cells around each newly filled cell. This results
        // in a heuristic map that works very well for the 3D search, as the 3D path
        // finding also only does "small moves" with a maximum distance of two cells
        // per iteration.
        // However, the 5D search can, at higher speed values, jump over more than
        // two cells at a time. In this case, it can happen that the heuristic value
        // slightly overestimates the true final cost of the cells. Therefore, we
        // subtract a small value from the pose's heuristic value to make sure the
        // value that is returned here is not an invalid heuristic value for the
        // A* search algorithm. (When testing, even with big maps, the error with
        // which the true costs were overestimated was always below 0.9.)
        return std::max(0.0f, heuristic->value(pose) - 0.9f);
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
    bool split_long_5d_segments,
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
    std::vector<Segment> segments = split_path(
        log_callback,
        map_resolution,
        costs,
        motion_model,
        split_long_5d_segments,
        path);
    print_segments(log_callback, segments);

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
