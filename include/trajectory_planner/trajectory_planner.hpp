#ifndef TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_HPP

#include "trajectory_planner/buffer.hpp"
#include "trajectory_planner/costs.hpp"

#include <tuple>
#include <vector>

namespace trajectory_planner
{

// A point, consisting of a X and Y coordinate, in meter.
using Point = std::tuple<double, double>;

// A (closed, first and last point are considered to be connected) polygon.
using Polygon = std::vector<Point>;

// A global planner that takes the robot's movement into account.
class TrajectoryPlanner
{
public:

    // Create a new planner.
    static std::unique_ptr<TrajectoryPlanner> create_planner(
        std::string hash,
        unsigned int angle_granularity,
        double resolution,
        const Polygon& footprint,
        Buffer<double> occupancy_map,
        Buffer<double> cost_map
    );

    // Return the original occupancy map this planner was created from.
    const Buffer<double>& original_occupancy_map() const;

    // Return the original cost map this planner was created from.
    const Buffer<double>& original_cost_map() const;

    // Return a value that is used to identify the costmap this
    // object was created from.
    const std::string& hash() const;

    // Write debug images that show the stored costs (one image for each orientation).
    void dump_orientation_maps(
        const std::string& prefix) const;

private:

    // Private constructure, use the factory function to create a new object.
    TrajectoryPlanner(
        std::string hash,
        Buffer<double> occupancy_map,
        Buffer<double> cost_map,
        Costs costs);

    // The hash of the costmap this planner was created from.
    const std::string hash_;

    // The occupancy map the planner was created from.
    const Buffer<double> original_occupancy_map_;

    // The cost map the planner was created from.
    const Buffer<double> original_cost_map_;

    // The costs for all valid poses.
    const Costs costs_;

    // Return the size, in pixels, of a canvas that is big enough to
    // hold the given footprint polygon in all orientations.
    static unsigned int calculate_footprint_size(
        const Polygon& footprint_polygon,
        double resolution);

    // Draw the footprint onto the given buffer. Returns the number of
    // covered pixels in the resulting image.
    static unsigned int draw_footprint(
        Buffer<double>& buffer,
        const Polygon& footprint_polygon,
        double resolution,
        int footprint_size,
        double theta);

    // Update the cost object with the values read from the given maps.
    static void update_costs(
        Costs& costs,
        size_t map_width,
        size_t map_height,
        size_t offset,
        unsigned int footprint_covered_pixels,
        const Buffer<double>& convoluted_occupancy_map,
        const Buffer<double>& convoluted_cost_map,
        unsigned int angle_index);
};

}

#endif
