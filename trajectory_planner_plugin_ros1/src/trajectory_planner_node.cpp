#include "trajectory_planner_plugin/trajectory_planner_plugin.hpp"

#include <tf2_ros/transform_listener.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trajectory_planner_node");

    ros::NodeHandle nh("~");

    tf2_ros::Buffer tf_buffer(ros::Duration(10));
    tf2_ros::TransformListener tf_listener(tf_buffer);
    costmap_2d::Costmap2DROS costmap("costmap", tf_buffer);

    trajectory_planner::TrajectoryPlannerPlugin planner_plugin;
    planner_plugin.initialize("TrajectoryPlanner", &costmap);

    auto path_pub = nh.advertise<nav_msgs::Path>("path", 1);

    boost::function<void(const geometry_msgs::PoseStamped&)> goal_callback =
        [&](const geometry_msgs::PoseStamped& goal_pose) -> void {
            geometry_msgs::PoseStamped start_pose;
            costmap.getRobotPose(start_pose);

            std::vector<geometry_msgs::PoseStamped> plan;
            if (!planner_plugin.makePlan(start_pose, goal_pose, plan)) {
                ROS_ERROR("Trajectory planner could not find a plan.");
                return;
            }

            nav_msgs::Path path_msg;
            path_msg.header.stamp = plan.at(0).header.stamp;
            path_msg.header.frame_id = plan.at(0).header.frame_id;
            std::copy(plan.cbegin(), plan.cend(), std::back_inserter(path_msg.poses));

            path_pub.publish(path_msg);
    };
    auto goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("goal", 1, goal_callback);

    ros::spin();
}
