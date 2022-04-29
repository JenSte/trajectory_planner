#ifndef TRAJECTORY_PLANNER_PLANNING_5D_HPP
#define TRAJECTORY_PLANNER_PLANNING_5D_HPP

#include "trajectory_planner/costs.hpp"

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
    // The path that was found for this segment.
    // TODO: make this a 5d path.
    Path path;

    // The cost object used for the search on this segment. This
    // is a subset of the global cost object, but only around the
    // 3D path of the segment.
    Costs costs;
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

// Inflate an given path, and return a new cost object that only
// contains the values around the path.
Costs inflate_path(
    const CircleCoordinatesMap& coordinates_map,
    unsigned int inflation_lookahead,
    const Costs& costs,
    const Path& path);

// Plan in the five dimensional space, using a three dimensional path to aid the search.
SearchResult5D plan(
    bool multi_threaded,
    unsigned int inflation_radius_pixels,
    unsigned int inflation_lookahead_pixels,
    const Costs& costs,
    const Path& path);
}

#endif
