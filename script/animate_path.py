#!/usr/bin/env python3

# This script listens for path messages and then publishes a footprint
# polygon, "animating" the movement of the robot on the path.

import math

import rclpy.node
import rclpy.qos
import transforms3d

from geometry_msgs.msg import Point32, PolygonStamped
from nav_msgs.msg import Path

import utils


class AnimatePath(rclpy.node.Node):

    def __init__(self, costmap_node, path_topic):
        super().__init__("animate_path")

        self._footprint = utils.get_footprint(self, costmap_node)
        self.get_logger().info(f"Costmap footprint: {self._footprint}")

        self._poses = []

        self._sub_path = self.create_subscription(
            Path,
            path_topic,
            self.path_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

        self._pub_polygon = self.create_publisher(
            PolygonStamped,
            "~/footprint_animation",
            rclpy.qos.qos_profile_system_default,
        )

        self._timer = None

    def path_callback(self, path):
        """Store the poses of the path for processing by the timer callback."""

        self._poses = path.poses

        self.get_logger().info(f"Got path with {len(self._poses)} poses.")

        if self._timer == None:
            self._timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        """Publish a polygon on the next path pose."""

        if not self._poses:
            self._timer.destroy()
            self._timer = None

            self.get_logger().info("Complete path processed.")
            return

        pose = self._poses.pop(0)

        # Get the orientation of the robot's footprint.
        angles = transforms3d.euler.quat2euler(
            (
                pose.pose.orientation.w,
                pose.pose.orientation.x,
                pose.pose.orientation.y,
                pose.pose.orientation.z,
            )
        )
        yaw = angles[2]

        # Rotate all the points in the footprint around the angle.
        s = math.sin(yaw)
        c = math.cos(yaw)
        footprint = [(x * c - y * s, x * s + y * c) for x, y in self._footprint]

        # Move the origin of the footprint to the current pose.
        dx = pose.pose.position.x
        dy = pose.pose.position.y
        footprint = [(x + dx, y + dy) for x, y in footprint]

        # Create polygon message from the points and publish it.
        polygon = PolygonStamped()
        polygon.header.frame_id = pose.header.frame_id
        for x, y in footprint:
            polygon.polygon.points.append(Point32(x=x, y=y))

        self._pub_polygon.publish(polygon)


def main(args=None):
    rclpy.init(args=args)

    animate_path = AnimatePath(
        "/global_costmap/global_costmap",
        "/planner_server/trajectory_planner/five_dimension_planner_path",
    )

    rclpy.spin(animate_path)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
