#ifndef TRAJECTORY_PLANNER_CONVERTER_HPP
#define TRAJECTORY_PLANNER_CONVERTER_HPP

#include "trajectory_planner/buffer.hpp"
#include "trajectory_planner/planning_3d.hpp"
#include "trajectory_planner/planning_5d.hpp"
#include "trajectory_planner/trajectory_planner.hpp"
#include "trajectory_planner/types.hpp"

#ifdef ROS1
#   include <costmap_2d/costmap_2d_ros.h>
#   include <geometry_msgs/Polygon.h>
#   include <nav_msgs/OccupancyGrid.h>
#   include <nav_msgs/Path.h>
#   include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#   include "trajectory_planner_msgs/AugmentedPath.h"
#else
#   include <nav2_util/costmap.hpp>
#   include <nav2_costmap_2d/costmap_2d_ros.hpp>
#   include <geometry_msgs/msg/polygon.hpp>
#   include <nav_msgs/msg/occupancy_grid.hpp>
#   include <nav_msgs/msg/path.hpp>
#   include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#   include "trajectory_planner_msgs/msg/augmented_path.hpp"
#endif

// This namespace contains functions to convert between ROS messages
// and the data structures used by the trajectory planner.
namespace trajectory_planner::convert
{

#if ROS_VERSION_MAJOR == 1
    using Costmap2D = costmap_2d::Costmap2D;
    using CostmapCostValue = unsigned char;
    using CostmapLockGuard = boost::lock_guard<boost::recursive_mutex>;
    using CostmapROSPointer = costmap_2d::Costmap2DROS*;

    using ROSOccupancyGrid = nav_msgs::OccupancyGrid;
    using ROSPath = nav_msgs::Path;
    using ROSPoint32 = geometry_msgs::Point32;
    using ROSPolygon = geometry_msgs::Polygon;
    using ROSPoseStamped = geometry_msgs::PoseStamped;
    using ROSTime = ros::Time;

    using TPAugmentedPath = trajectory_planner_msgs::AugmentedPath;
    using TPAugmentedPose = trajectory_planner_msgs::AugmentedPose;
    using TPMotionModel = trajectory_planner_msgs::MotionModel;
#else
    using Costmap2D = nav2_costmap_2d::Costmap2D;
    using CostmapCostValue = nav2_util::Costmap::CostValue;
    using CostmapLockGuard = std::lock_guard<std::recursive_mutex>;
    using CostmapROSPointer = std::shared_ptr<nav2_costmap_2d::Costmap2DROS>;

    using ROSOccupancyGrid = nav_msgs::msg::OccupancyGrid;
    using ROSPath = nav_msgs::msg::Path;
    using ROSPoint32 = geometry_msgs::msg::Point32;
    using ROSPolygon = geometry_msgs::msg::Polygon;
    using ROSPoseStamped = geometry_msgs::msg::PoseStamped;
    using ROSTime = rclcpp::Time;

    using TPAugmentedPath = trajectory_planner_msgs::msg::AugmentedPath;
    using TPAugmentedPose = trajectory_planner_msgs::msg::AugmentedPose;
    using TPMotionModel = trajectory_planner_msgs::msg::MotionModel;
#endif

// Create a ROS timestamp object matching the ROS version.
ROSTime ros_time(
    double seconds);

// Convert a ROS polygon message to a polygon object used by the trajectory planner.
trajectory_planner::Polygon polygon_msg_to_polygon(
    const ROSPolygon& polygon_msg);

// Create a buffer that contains the value "1.0" where there are objects
// in the given costmap.
Buffer<double> create_occupancy_map(
    const Costmap2D* costmap);

// Convert a occupancy map as used by the trajectory planner to a ROS OccupancyGrid.
ROSOccupancyGrid buffer_to_occupancy_grid_msg(
    const Buffer<double>& buffer,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y);

// Convert a ROS stamped pose message to a pose object used by the trajectory planner.
Pose pose_msg_to_pose(
    const ROSPoseStamped& pose_msg,
    const Costmap2D* costmap,
    unsigned int angle_granularity);

// Convert a pose structure as used by the trajectory planner to a ROS pose message.
ROSPoseStamped pose_to_pose_msg(
    const Costmap2D* costmap,
    const std::string& frame_id,
    double timestamp,
    const Pose& pose,
    unsigned int angle_granularity);

// Convert a pose structure as used by the 3D planner to a ROS pose message.
ROSPoseStamped pose3d_to_pose_msg(
    const Costmap2D* costmap,
    const std::string& frame_id,
    double timestamp,
    const three::Pose3D& pose_3d);

// Create a path message for a 3D search result.
ROSPath plan_3d_to_path_msg(
    const Costmap2D* costmap,
    const std::string& costmap_frame_id,
    const TrajectoryPlanner::Result& result);

// Convert the heuristic used by the 3d planner to an occupancy grid ROS message.
ROSOccupancyGrid heuristic_3d_to_occupancy_grid_msg(
    const DepthHeuristic& heuristic,
    size_t width,
    size_t height,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y);

// Create a ROS occupancy grid from the nodes opened by the 3D search.
ROSOccupancyGrid opened_3d_nodes_to_occupancy_grid_msg(
    const three::SearchResult3D::OpenedNodesMap& opened_nodes,
    size_t width,
    size_t height,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y);

// Create an occupancy grid ROS message that shows the search space of the 5D planner.
ROSOccupancyGrid search_space_5d_to_occupacy_grid_msg(
    const std::vector<five::SegmentSearchResult>& segments,
    size_t width,
    size_t height,
    const std::string& frame_id,
    double resolution,
    double origin_x,
    double origin_y);

// Create an augmented path message for a 3D search result.
TPAugmentedPath search_result_3d_to_augmented_path_msg(
    const std::string& name,
    const unsigned segment_count,
    const unsigned total_segments,
    const Costmap2D* costmap_2d,
    const std::string& frame_id,
    const MotionModel& motion_model,
    const three::SearchResult3D& search_result);

// Create an augmented path message for a 5D segment search result.
TPAugmentedPath search_result_5d_to_augmented_path_msg(
    const std::string& name,
    const unsigned segment_count,
    const unsigned total_segments,
    const Costmap2D* costmap_2d,
    const std::string& frame_id,
    unsigned int angle_granularity,
    const MotionModel& motion_model,
    const five::SegmentSearchResult& search_result);

// Create a motion model ROS message.
TPMotionModel motion_model_to_motion_model_msg(
    const MotionModel& motion_model);

// Create a ROS path message from the 5D search result.
ROSPath search_result_5d_to_path_msg(
    const Costmap2D* costmap_2d,
    const std::string& costmap_frame_id,
    unsigned int angle_granularity,
    const TrajectoryPlanner::Result& result);

}

#endif
