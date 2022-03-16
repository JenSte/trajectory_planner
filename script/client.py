#!/usr/bin/env python3

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from nav2_msgs.action import ComputePathToPose

class ComputePathActionClient(Node):

    def __init__(self):
        super().__init__("compute_path_action_client")
        self._action_client = ActionClient(self, ComputePathToPose, "compute_path_to_pose")

    def send_goal(self):
        goal_msg = ComputePathToPose.Goal()

        goal_msg.start.header.frame_id = "map"
        goal_msg.start.pose.position.x = 0.5
        goal_msg.start.pose.position.y = 0.6
        goal_msg.start.pose.position.z = 0.0

        goal_msg.goal.header.frame_id = "map"
        goal_msg.goal.pose.position.x = 1.5
        goal_msg.goal.pose.position.y = 1.6
        goal_msg.goal.pose.position.z = 0.0

        goal_msg.planner_id = "MultidimensionPlanner"
        goal_msg.use_start = True

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.path.poses))
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    action_client = ComputePathActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
