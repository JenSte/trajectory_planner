#!/usr/bin/env python3

# This script subscribes to the ground-truth topic and fakes a localization.

import math

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from tf2_ros import TransformBroadcaster
import rclpy


class FakeLocalization(Node):

    def __init__(self):
        super().__init__('ground_truth_localization')

        # The last received odometry pose from the "odom" topic.
        self._last_odom_pose = None

        self.odometry_subscription = self.create_subscription(
            Odometry, 'odom', self.odom_callback, 10
        )

        self.ground_truth_subscription = self.create_subscription(
            Odometry, 'ground_truth', self.ground_truth_callback, 10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

    def odom_callback(self, msg):
        self._last_odom_pose = msg.pose.pose

    def ground_truth_callback(self, msg):
        if self._last_odom_pose is None:
            return

        odom_pose = self._last_odom_pose
        gt_pose = msg.pose.pose

        _, _, odom_yaw = euler_from_quaternion(
            [odom_pose.orientation.x, odom_pose.orientation.y, odom_pose.orientation.z, odom_pose.orientation.w]
        )
        _, _, gt_yaw = euler_from_quaternion(
            [gt_pose.orientation.x, gt_pose.orientation.y, gt_pose.orientation.z, gt_pose.orientation.w]
        )

        x, y, theta = self.map_to_odom(
            gt_pose.position.x, gt_pose.position.y, gt_yaw,
            odom_pose.position.x, odom_pose.position.y, odom_yaw,
        )

        q = quaternion_from_euler(0.0, 0.0, theta)
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        self.tf_broadcaster.sendTransform(t)

    def map_to_odom(self, x, y, th, ox, oy, oth):
        th_mo = (th - oth) % (2 * math.pi)
    
        c = math.cos(th_mo)
        s = math.sin(th_mo)
    
        x_mo = x - (c * ox - s * oy)
        y_mo = y - (s * ox + c * oy)
    
        return x_mo, y_mo, th_mo


if __name__ == "__main__":
    try:
        rclpy.init()
        rclpy.spin(FakeLocalization())
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
