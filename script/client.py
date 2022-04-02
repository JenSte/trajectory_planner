#!/usr/bin/env python3

# This script connects to the action interface of planner server and
# requests a path between two poses.

import math

import rclpy
import rclpy.action
import rclpy.node
import transforms3d

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose


class ComputePathActionClient(rclpy.node.Node):

    def __init__(self):
        super().__init__("compute_path_action_client")
        self._action_client = rclpy.action.ActionClient(
            self, ComputePathToPose, "compute_path_to_pose")

    def send_goal(self):
        goal_msg = ComputePathToPose.Goal()

        goal_msg.start = self.make_pose("map", 1.5, 1.5, math.radians(-5.0))
        goal_msg.goal = self.make_pose("map", 1.5, 4.5, math.radians(0.0))

        goal_msg.planner_id = "trajectory_planner"
        goal_msg.use_start = True

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info("Goal was rejected.")
            return

        self.get_logger().info("Goal was accepted.")

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result

        if not result.path.poses:
            self.get_logger().info("Returned path is empty.")
        else:
            self.get_logger().info("Path:")
            self.log_path(result.path)

        rclpy.shutdown()

    def make_pose(self, frame_id, x, y, yaw):
        """Create a ROS PoseStamped message."""

        q = transforms3d.euler.euler2quat(0, 0, yaw)

        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = q[0]
        pose.pose.orientation.x = q[1]
        pose.pose.orientation.y = q[2]
        pose.pose.orientation.z = q[3]

        return pose

    def log_path(self, path):
        """Log the path."""

        self.get_logger().info("         X       Y        θ")
        for i, p in enumerate(path.poses):
            angles = transforms3d.euler.quat2euler(
                (
                    p.pose.orientation.w,
                    p.pose.orientation.x,
                    p.pose.orientation.y,
                    p.pose.orientation.z,
                )
            )

            x = p.pose.position.x
            y = p.pose.position.y
            yaw = math.degrees(angles[2] % (2 * math.pi))

            self.get_logger().info(f"{i:4}:  {x:5.2f} / {y:5.2f} / {yaw:5.1f}")


def main(args=None):
    rclpy.init(args=args)

    action_client = ComputePathActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
