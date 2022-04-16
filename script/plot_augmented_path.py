#!/usr/bin/env python3

# This script listens to the AugmentedPath messages published by the
# trajectory planner and plots the data contained. This plot is mainly
# interresting for debugging the search algorithm.

import collections
import math
import pathlib

import matplotlib
import matplotlib.figure
import matplotlib.pyplot
matplotlib.use("agg")

import rclpy.node
import rclpy.qos
import transforms3d

from trajectory_planner.msg import AugmentedPath


# A few constants that influence the size of the plot.
WIDTH = 1800
HEIGHT = 1250
DPI = 100


class PlotAugmentedPath(rclpy.node.Node):

    def __init__(self, augmented_path_topic, output_directory):
        super().__init__("plot_augmented_path")

        self._output_directory = output_directory

        self._sub_augmented_path = self.create_subscription(
            AugmentedPath,
            augmented_path_topic,
            self.augmented_path_callback,
            rclpy.qos.qos_profile_sensor_data,
        )

    def augmented_path_callback(self, path):
        """Plot the details of the augmented path."""

        DataPoint = collections.namedtuple(
            "DataPoint", ["x", "y", "theta", "cost", "heuristic"]
        )

        def extract_yaw(quaternion):
            """Extract the ROS quaternion message."""

            angles = transforms3d.euler.quat2euler(
                (quaternion.w, quaternion.x, quaternion.y, quaternion.z)
            )

            return angles[2] % (2 * math.pi)

        def extract_data_point(augmented_pose):
            return DataPoint(
                augmented_pose.pose.pose.position.x,
                augmented_pose.pose.pose.position.y,
                extract_yaw(augmented_pose.pose.pose.orientation),
                augmented_pose.cost,
                augmented_pose.heuristic,
            )

        data = [extract_data_point(pose) for pose in path.poses]
        self.plot(data, path.name)

    def plot(self, data, name):
        """Create a plot and save it."""

        time = range(len(data))

        subplots = 4

        fig = matplotlib.figure.Figure(figsize=(WIDTH / DPI, HEIGHT / DPI), dpi=DPI)
        fig.suptitle(name, fontsize=20)

        axes_x = fig.add_subplot(subplots, 1, 1)
        axes_x.plot(time, [d.x for d in data])
        axes_x.set_ylabel("X\n(m)")
        axes_x.grid(True)

        axes_y = fig.add_subplot(subplots, 1, 2, sharex=axes_x)
        axes_y.plot(time, [d.y for d in data])
        axes_y.set_ylabel("Y\n(m)")
        axes_y.grid(True)

        axes_t = fig.add_subplot(subplots, 1, 3, sharex=axes_x)
        axes_t.plot(time, [d.theta for d in data])
        axes_t.set_ylabel("$\\theta$\n(rad)")
        axes_t.grid(True)

        # Costs and heuristic values on one plot.
        axes_c = fig.add_subplot(subplots, 1, 4, sharex=axes_x)
        axes_c.scatter(time, [d.heuristic for d in data], marker="x")
        axes_c.scatter(time, [d.cost for d in data], marker="+", color="r")
        axes_c.set_ylabel("heuristic (blue)\npath cost (red)")
        axes_c.set_xlabel("time (sec)")
        axes_c.grid(True)

        matplotlib.pyplot.setp(axes_x.get_xticklabels(), visible=False)
        matplotlib.pyplot.setp(axes_y.get_xticklabels(), visible=False)
        matplotlib.pyplot.setp(axes_t.get_xticklabels(), visible=False)

        filepath = str((self._output_directory / name).with_suffix(".png"))
        fig.savefig(filepath, bbox_inches="tight")
        self.get_logger().info(f"Plot saved to '{filepath}'.")


def main(args=None):
    rclpy.init(args=args)

    plot_augmented_path = PlotAugmentedPath(
        "/planner_server/trajectory_planner/augmented_path",
        pathlib.Path.cwd(),
    )

    rclpy.spin(plot_augmented_path)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
