#ifndef TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_HPP

#include "trajectory_planner/buffer.hpp"
#include "trajectory_planner/costs.hpp"
#include "trajectory_planner/planning_3d.hpp"
#include "trajectory_planner/types.hpp"

#include <iostream>
#include <functional>

namespace trajectory_planner
{

// A global planner that takes the robot's movement into account.
class TrajectoryPlanner
{
public:

    // Type of a callback function used to output log messages.
    using LogCallback = std::function<void(const std::string&)>;

    // The result returned by a call to 'plan()'.
    struct Result
    {
        // The path (if found, otherwise empty).
        Path path;

        // The search result of the three dimensional planner
        // used to get a rough estimate for the final path.
        three::SearchResult3D search_result_3d;
    };

    // Create a new planner.
    static std::unique_ptr<TrajectoryPlanner> create_planner(
        LogCallback log_callback,
        bool multi_threaded,
        std::string hash,
        unsigned int angle_granularity,
        double resolution,
        double inflation_radius,
        const Polygon& footprint,
        Buffer<double> occupancy_map
    );

    // Load a planner from a file created by 'store()'.
    static std::unique_ptr<TrajectoryPlanner> load_planner(
        LogCallback log_callback,
        std::string hash,
        Buffer<double> occupancy_map,
        std::istream& istream);

    // Return the original occupancy map this planner was created from.
    const Buffer<double>& original_occupancy_map() const;

    // Return the cost map this planner is using.
    const Buffer<double>& cost_map() const;

    // Return a value that is used to identify the map this object was created from.
    const std::string& hash() const;

    // Return the angle granularity used to create this planner object.
    unsigned int angle_granularity() const;

    // Write debug images that show the stored costs (one image for each orientation).
    void dump_orientation_maps(
        const std::string& prefix) const;

    // Plan a route from 'start' to 'goal'.
    Result plan(
        const Pose& start,
        const Pose& goal) const;

    // Store what is necessary to restore the planner to a given file stream.
    void store(
        std::ostream& ostream) const;

private:

    // Private constructor, use the factory function to create a new object.
    TrajectoryPlanner(
        LogCallback log_callback,
        std::string hash,
        Buffer<double> occupancy_map,
        Buffer<double> cost_map,
        Costs costs);

    // Create the "cost map" by inflating the occupancy map, using an exponential
    // decay function.
    //
    // The reason for creating the cost values on our own and not using the one
    // provided by ROS' costmap_2d is that costmap_2d derives an "inscribed radius"
    // from the robot's footprint. This is the smallest distance the robot can be
    // placed next to an obstacle. All cells in the costmap_2d within the range of
    // this inscribed radius to an obstacle are then set to the same constant value
    // and the decay function only starts around that area. Because our per-pose cost
    // extraction function (see 'create_costs()') "integrates" up all the cost values
    // covered by the footprint using the values from costmap_2d resultd in non-optimal
    // costs, especially around corners. Therefore this function implements the costmap
    // calculation without an inscribed radius, starting the inflation directly around
    // obstacles on the occupancy map.
    static Buffer<double> create_cost_map(
        const LogCallback& log_callback,
        bool multi_threaded,
        double resolution,
        double inflation_radius,
        const Buffer<double>& occupancy_map);

    // Create an objects holding the costs of placing the robot's footprint
    // on the map in any direction.
    static Costs create_costs(
        const LogCallback& log_callback,
        bool multi_threaded,
        unsigned int angle_granularity,
        double resolution,
        const Polygon& footprint,
        const Buffer<double>& occupancy_map,
        const Buffer<double>& cost_map);

    // Callback to do logging.
    const LogCallback log_callback_;

    // The hash of the costmap this planner was created from.
    const std::string hash_;

    // The occupancy map the planner was created from.
    const Buffer<double> original_occupancy_map_;

    // The cost map of this planner.
    const Buffer<double> cost_map_;

    // The costs for all valid poses.
    const Costs costs_;

    // Return the size, in pixels, of a canvas that is big enough to
    // hold the given footprint polygon in all orientations.
    static unsigned int calculate_footprint_size(
        const Polygon& footprint_polygon,
        double resolution);

    // Write a notification text on a buffer.
    static void draw_text(
        Buffer<double>& buffer,
        const std::string& text);

    // Draw a circle on a buffer. The center of the circle is placed at half the
    // canvas size.
    static void draw_circle(
        Buffer<double>& buffer,
        int canvas_size,
        int radius);

    // Draw the footprint onto the given buffer. Returns the number of
    // covered pixels in the resulting image.
    static unsigned int draw_footprint(
        Buffer<double>& buffer,
        const Polygon& footprint_polygon,
        double resolution,
        int footprint_size,
        double theta);

    // Get the cost values from the given maps.
    static std::vector<std::tuple<size_t, size_t, double>> extract_costs(
        size_t map_width,
        size_t map_height,
        size_t offset,
        unsigned int footprint_covered_pixels,
        const Buffer<double>& convoluted_occupancy_map,
        const Buffer<double>& convoluted_cost_map);
};

}

#endif
