#ifndef TRAJECTORY_PLANNER_PLANNING_5D_HPP
#define TRAJECTORY_PLANNER_PLANNING_5D_HPP

#include "trajectory_planner/costs.hpp"
#include "trajectory_planner/heuristic.hpp"
#include "trajectory_planner/motion_model.hpp"

// The functions in this name space implement a "five dimensional" search. This
// means that the search happens in on cells (X, Y, Theta, linear velodicty, angular
// velocity), i.e. taking the pose and the speeds of the robot into account.
namespace trajectory_planner::five
{

// Describes the direction of a path segment.
enum class Direction {
    // The poses in the segment move forward (positive linear velocity).
    FORWARD,

    // The poses in the segment move backward (negative linear velocity).
    BACKWARD,

    // The poses stay on the same spot, and only change the orientation.
    TURN
};

// A segment is a part of the 3D path that contains poses where
// the movement is only in a single direction (forward or backward)
// or where the robot turns on a spot.
struct Segment
{
    // The directon of the segment.
    Direction direction;

    // The poses that make up this segment.
    Path path;
};

// The search result for a single segment.
struct SegmentSearchResult
{
    // The directon of the segment.
    Direction direction;

    // The path that was found for this segment.
    Path5D path;

    // Contains the heuristic value for every pose in the path (vector has
    // the exact same length as 'path', not used for TURN segments).
    std::vector<double> path_heuristics;

    // The cost values of the poses along the path (vector has the exact same
    // length as 'path', not used for TURN segments).
    std::vector<double> path_costs;

    // The cost object used for the search on this segment. This
    // is a subset of the global cost object, but only around the
    // 3D path of the segment.
    Costs costs;

    // The heuristic object used for planning this segment.
    std::unique_ptr<Heuristic> heuristic;
};

// Data type returned from the search in 5D space.
struct SearchResult5D
{
    // The results of the searches on the individual segments of the 3D path.
    std::vector<SegmentSearchResult> segment;
};

// Split a path into multiple segments with different directions.
std::vector<Segment> split_path(
    unsigned int angle_granularity,
    const Path& path);

// Coordinates covered by a circle which has the origin at 0/0.
using CircleCoordinates = std::vector<std::tuple<int, int>>;

// Type that maps a circles radius to the coordinates that circle covers.
using CircleCoordinatesMap = std::map<unsigned int, CircleCoordinates>;

// Calculate the coordinates a circle with a given radius covers.
CircleCoordinates circle_coordinates(
    unsigned int pixel_radius);

// Return a heuristic object matching the given type.
std::unique_ptr<Heuristic> create_heuristic(
    HeuristicType heuristic_type,
    const Pose5D& goal,
    const Path& path,
    const Costs& search_space);

// Inflate an given path, and return a new cost object that only
// contains the values around the path.
Costs inflate_path(
    const CircleCoordinatesMap& coordinates_map,
    unsigned int inflation_lookahead,
    const Costs& costs,
    const Path& path);

// Calculate the distance that is traveled from "pose" to "neighbour". Basically, doing
// this calculation would should be as easy as using the Pythagorean theorem on the gris
// coordinates of the two poses. However, because of the rounding needed to map the
// calculated distances (floating point numbers) to the cells of the map's grid, it can
// happen that multiple velocity combination from a given start coordinate map to the
// same goal coordinate (on the map's grid). To handle this, this function returns a
// tuple consisting of three distances: The easy to calculate the distance when using
// the map cell coordinates, the distance when using the velocities without rounding,
// and the error distance between the first two ones (calculated as vector lengths).
std::tuple<double, double, double, double> calculate_movement_distances(
    double map_resolution,
    unsigned int angle_granularity,
    const MotionModel& motion_model,
    const Pose5D& pose,
    const Pose5D& neighbour);

// Plan in the five dimensional space, using a three dimensional path to aid the search.
SearchResult5D plan(
    bool multi_threaded,
    double map_resolution,
    unsigned int inflation_radius_pixels,
    unsigned int inflation_lookahead_pixels,
    HeuristicType heuristic_type,
    const Costs& costs,
    const MotionModel& motion_model,
    const Path& path);
}

#endif
