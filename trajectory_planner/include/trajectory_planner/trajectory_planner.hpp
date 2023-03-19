#ifndef TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_HPP
#define TRAJECTORY_PLANNER_TRAJECTORY_PLANNER_HPP

#include "trajectory_planner/buffer.hpp"
#include "trajectory_planner/costs.hpp"
#include "trajectory_planner/planning_3d.hpp"
#include "trajectory_planner/planning_5d.hpp"
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

    // Contains additional parameters that are passed to the planning function.
    struct PlanningParameters
    {
        // The maximum radius to inflate the 3D path up when calculating
        // the 5D search space, in meter.
        double inflation_5d_radius;

        // The maximum distance to look forward/backward on the 3D path when
        // "measuring" the curvature, in meter.
        double inflation_5d_lookahead;

        // The maximum velocity of a wheel of the vehicle, in meter/second.
        double maximum_wheel_velocity;

        // The maximum acceleration of a wheel of the vehicle, in meter/(second^2).
        double maximum_wheel_acceleration;

        // The distance between the two driving wheels of the vehicle, in meter.
        double wheel_distance;

        // The maximum linear velocity of the vehicle, in meter/second.
        //double linear_velocity_maximum;

        // The maximum angular velocity of the vehicle, in radian/second.
        //double angular_velocity_maximum;

        // The number of steps to subdivide 'linear_velocity_maximum'.
        unsigned int linear_velocity_steps;

        // The number of steps to subdivide 'angular_velocity_maximum'.
        unsigned int angular_velocity_steps;

        // The maximum linear acceleration of the vehicle, in meter/(second^2).
        double linear_acceleration_maximum;

        // The maximum angular acceleration of the vehicle, in radian/(second^2).
        double angular_acceleration_maximum;

        // The time increment used during the simulation, in second.
        //double time_delta;

        // Wheter or not to do a multi-threaded search.
        bool multi_threaded;

        // The heuristic to use for the 5D search.
        HeuristicType heuristic_type;
    };

    // The result returned by a call to 'plan()'.
    struct Result
    {
        // The search result of the three dimensional planner
        // used to get a rough estimate for the final path.
        three::SearchResult3D search_result_3d;

        // The search result fo the five dimensional planner.
        five::SearchResult5D search_result_5d;

        // The motion model used to do the five dimensional planning.
        MotionModel motion_model;
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
        Buffer<double> occupancy_map,
        std::optional<Pose2D> internal_point);

    // Load a planner from a file created by 'store()'.
    static std::unique_ptr<TrajectoryPlanner> load_planner(
        LogCallback log_callback,
        double resolution,
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
    void write_debug_images(
        const std::string& prefix) const;

    // Write debug images that show the details of the 3D and 5D search results.
    void write_result_debug_images(
        const Result& result,
        const std::string& prefix) const;

    // Plan a route from 'start' to 'goal'.
    Result plan(
        const PlanningParameters& parameters,
        const Pose& start,
        const Pose& goal) const;

    // Store what is necessary to restore the planner to a given file stream.
    void store(
        std::ostream& ostream) const;

private:

    // Private constructor, use the factory function to create a new object.
    TrajectoryPlanner(
        LogCallback log_callback,
        double map_resolution,
        std::string hash,
        Buffer<double> occupancy_map,
        Buffer<double> cost_map,
        Costs costs);

    // Modify the map so that it only contains the free space around 'point'
    // (coordinates in raw pixel coordinates). Other "rooms" that are not connected
    // by free pixels with 'point' are marked as inaccessible.
    static void crop_outer(
        const LogCallback& log_callback,
        Pose2D point,
        Buffer<double>& map);

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

    // The resolution (in meter/pixel) of the maps.
    const double map_resolution_;

    // The hash of the costmap this planner was created from.
    const std::string hash_;

    // The occupancy map the planner was created from.
    const Buffer<double> original_occupancy_map_;

    // The cost map of this planner.
    const Buffer<double> cost_map_;

    // The costs for all valid poses.
    const Costs costs_;

    // Check if a pose is valid, raise an exception if not.
    void check_pose(
        const Pose& pose,
        const std::string& pose_name) const;

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

    // Get the 3D cost values from the given maps.
    static Costs::CostVector3D extract_costs_3d(
        const Costs& costs,
        size_t map_width,
        size_t map_height,
        size_t offset,
        unsigned int footprint_covered_pixels,
        const Buffer<double>& convoluted_occupancy_map,
        const Buffer<double>& convoluted_cost_map);

    // Get the 5D cost (occupancy) values from the given map.
    static Costs::CostVector5D extract_costs_5d(
        const Costs& costs,
        size_t map_width,
        size_t map_height,
        size_t offset,
        const Buffer<double>& convoluted_occupancy_map);

    // Print some details of the motion model to the log.
    void log_motion_model(
        const MotionModel& motion_model) const;
};

}

#endif
