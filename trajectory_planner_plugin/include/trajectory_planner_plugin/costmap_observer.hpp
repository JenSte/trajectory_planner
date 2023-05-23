#ifndef TRAJECTORY_PLANNER_COSTMAP_OBSERVER_HPP
#define TRAJECTORY_PLANNER_COSTMAP_OBSERVER_HPP

#include "trajectory_planner/trajectory_planner.hpp"
#include "trajectory_planner_plugin/convert.hpp"

#include <condition_variable>
#include <mutex>
#include <thread>

namespace trajectory_planner
{

// Observes a costmap and refreshes the planner on changes.
class CostmapObserver
{
public:

    // Type of a callback to log a message.
    using LogCallback = std::function<void(const std::string&)>;

    // Type of a callback to publish a OccupancyGrid message.
    using OccupancyGridCallback = std::function<void(const convert::ROSOccupancyGrid&)>;

    CostmapObserver(
        LogCallback log_info,
        OccupancyGridCallback original_occupancy_map_callback,
        OccupancyGridCallback cost_map_callback);

    CostmapObserver(const CostmapObserver&) = delete;
    CostmapObserver& operator=(const CostmapObserver&) = delete;
    CostmapObserver(CostmapObserver&&) = delete;
    CostmapObserver& operator=(CostmapObserver&&) = delete;

    ~CostmapObserver();

    // Start watching a given costmap for changes.
    void watch_costmap(
        trajectory_planner::three::CostMap3DType cost_map_3d_type,
        trajectory_planner::three::OrientationCosts3DType orientation_costs_3d_type,
        int angle_granularity,
        double inscribed_radius,
        double inflation_radius,
        const std::string& cache_directory,
        const std::string& debug_directory,
        std::optional<Point> internal_point,
        convert::CostmapROSPointer costmap);

    // Stop watching the costmap.
    void stop_watching();

    // Return a pointer to the planner.
    std::shared_ptr<TrajectoryPlanner> planner() const;

private:

    // Watches the costmap for changes and (re-)creates the planner if necessary.
    void planner_update_thread_function(
        trajectory_planner::three::CostMap3DType cost_map_3d_type,
        trajectory_planner::three::OrientationCosts3DType orientation_costs_3d_type,
        int angle_granularity,
        double inscribed_radius,
        double inflation_radius,
        std::string cache_directory,
        std::string debug_directory);

    // Create a hash used to identify the costmap.
    std::string hash_costmap(
        trajectory_planner::three::CostMap3DType cost_map_3d_type,
        trajectory_planner::three::OrientationCosts3DType orientation_costs_3d_type,
        int angle_granularity,
        double inscribed_radius,
        double inflation_radius,
        const Polygon& footprint,
        const convert::Costmap2D* costmap) const;

    // Return the file name that shall be used to cache a planner's cost object.
    std::string cache_file_name(
        const std::string& cache_directory,
        const std::string& hash) const;

    // Load a planner from a given file.
    std::unique_ptr<TrajectoryPlanner> load_planner(
        const std::string& cache_file,
        const std::string& hash,
        const convert::Costmap2D* costmap_2d) const;

    // Create a planner.
    std::unique_ptr<TrajectoryPlanner> create_planner(
        trajectory_planner::three::CostMap3DType cost_map_3d_type,
        trajectory_planner::three::OrientationCosts3DType orientation_costs_3d_type,
        const std::string& hash,
        int angle_granularity,
        double inscribed_radius,
        double inflation_radius,
        const Polygon& footprint,
        const convert::Costmap2D* costmap_2d) const;

    // Store the planner in the given file.
    void store_planner(
        const std::string& cache_file,
        const TrajectoryPlanner* planner) const;

    // Write debug images to the given directory.
    void write_debug_images(
        const std::string& debug_directory,
        const TrajectoryPlanner* planner) const;

    // Callback to log a message with "info" verbosity.
    LogCallback log_info_;

    // Callback to publish the occupancy grid the planner received.
    OccupancyGridCallback original_occupancy_map_callback_;

    // Callback to publish the costmap the planner creates.
    OccupancyGridCallback cost_map_callback_;

    // An optional coordinate which, if given, defines a point inside the
    // room to plan. Used to crop out that room, to make planning more efficient.
    std::optional<Point> internal_point_;

    // The costmap this object is watching.
    convert::CostmapROSPointer costmap_ros_;

    // A pointer to the planner.
    std::shared_ptr<TrajectoryPlanner> planner_;

    // Background thread that periodically checks for changes of the costmap
    // and updates the planner if necessary.
    std::thread planner_update_thread_;

    // Mutex to be used together with 'planner_update_thread_notification_'.
    std::mutex planner_update_thread_mutex_;

    // Condition variable to communicate with the planner update thread.
    std::condition_variable planner_update_thread_notification_;

    // Set to 'true' to notify the background thread to stop.
    bool planner_update_thread_quit_ = false;
};

}

#endif
