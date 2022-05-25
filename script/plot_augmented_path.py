#!/usr/bin/env python3

# This script listens to the AugmentedPath messages published by the
# trajectory planner and plots the data contained. This plot is mainly
# interresting for debugging the search algorithm.

import collections
import functools
import math
import pathlib
import re

import matplotlib
import matplotlib.figure
import matplotlib.pyplot
import matplotlib.ticker
matplotlib.use("agg")

import rclpy.node
import rclpy.qos
import transforms3d

from trajectory_planner.msg import AugmentedPath

import utils


# A few constants that influence the size of the plot.
WIDTH = 2000
HEIGHT = 1550
DPI = 100


# Named tuple to hold the individual data points of a 5D path.
DataPoint = collections.namedtuple(
    "DataPoint", [
        "time",
        "x",
        "y",
        "theta",
        "linear_index",
        "angular_index",
        "cost",
        "heuristic",
    ]
)

def _extract_yaw(quaternion):
    """Extract the ROS quaternion message."""

    angles = transforms3d.euler.quat2euler(
        (quaternion.w, quaternion.x, quaternion.y, quaternion.z)
    )

    return angles[2] % (2 * math.pi)


def _extract_timestamp(timestamp):
    """Generate a floating point second value from a ROS stamp message."""

    return timestamp.sec + timestamp.nanosec / 1e9


def extract_data_point(augmented_pose):
    """Create a 'DataPoint' object from an augmented pose ROS message."""

    return DataPoint(
        _extract_timestamp(augmented_pose.pose.header.stamp),
        augmented_pose.pose.pose.position.x,
        augmented_pose.pose.pose.position.y,
        _extract_yaw(augmented_pose.pose.pose.orientation),
        augmented_pose.linear_index,
        augmented_pose.angular_index,
        augmented_pose.cost,
        augmented_pose.heuristic,
    )


def create_luts(motion_model):
    """Create lookup-tables that map velocity indices to their actual values."""

    linear_velocity_lut = {}
    for i, v in enumerate(motion_model.linear_velocities):
        linear_velocity_lut[i] = v
        if i != 0:
            linear_velocity_lut[-i] = -v

    angular_velocity_lut = {}
    for i, v in enumerate(motion_model.angular_velocities):
        angular_velocity_lut[i] = v
        if i != 0:
            angular_velocity_lut[-i] = -v

    return linear_velocity_lut, angular_velocity_lut


def _linear_labels(linear_velocity_lut, y, pos):
    i = round(y)
    if abs(i - y) > 0.001:
        return ""

    v = linear_velocity_lut.get(i)
    if v is None:
        return ""

    return f"{int(y)}\n{v:.02f} m/s"


def _angular_labels(angular_velocity_lut, y, pos):
    i = round(y)
    if abs(i - y) > 0.001:
        return ""

    v = angular_velocity_lut.get(y)
    if v is None:
        return ""

    return f"{int(y)}\n{v:.02f} rad/s"


class PlotAugmentedPath(rclpy.node.Node):

    def __init__(self, augmented_path_topic, costmap_node, output_directory):
        super().__init__("plot_augmented_path")

        self._output_directory = output_directory

        self._sub_augmented_path = self.create_subscription(
            AugmentedPath,
            augmented_path_topic,
            self.augmented_path_callback,
            10,
        )

        footprint = utils.get_footprint(self, costmap_node)
        self._wheel_distance = utils.estimate_wheel_distance(footprint)
        self.get_logger().info(f"Estimated wheel distance: {self._wheel_distance:.3f} m")

    def augmented_path_callback(self, path):
        """Plot the details of the augmented path."""

        linear_velocity_lut, angular_velocity_lut = create_luts(path.motion_model)
        # self.get_logger().info(f"linear velocities: {linear_velocity_lut}")
        # self.get_logger().info(f"angular velocities: {angular_velocity_lut}")

        data = [extract_data_point(pose) for pose in path.poses]
        self.plot_over_time(
            data, path.name, linear_velocity_lut, angular_velocity_lut, self._wheel_distance
        )

        if "_5d_" in path.name:
            self.plot_motion_model(
                data, path.name, path.motion_model, linear_velocity_lut, angular_velocity_lut
            )

    def plot_over_time(self, data, name, linear_velocity_lut, angular_velocity_lut, wheel_distance):
        """Create a plot (with a number of subplots) that show the path's details
        over the time."""

        # Determine if we plot a five dimensional path by looking at the name.
        five = "_5d_" in name

        if five:
            time = [d.time for d in data]
            subplots = 7
        else:
            # 3D paths do not include timestamps for the poses.
            time = range(len(data))
            subplots = 4

        right_wheel_speeds = []
        left_wheel_speeds = []
        for d in data:
            linear_velocity = linear_velocity_lut[d.linear_index]
            angular_velocity = angular_velocity_lut[d.angular_index]

            left_wheel_speeds.append(linear_velocity - wheel_distance * angular_velocity / 2.0)
            right_wheel_speeds.append(linear_velocity + wheel_distance * angular_velocity / 2.0)

        # The number of the current plot.
        plot = 0

        fig = matplotlib.figure.Figure(figsize=(WIDTH / DPI, HEIGHT / DPI), dpi=DPI)
        fig.suptitle("Path Details", fontsize=20)

        plot += 1
        axes_x = fig.add_subplot(subplots, 1, plot)
        axes_x.plot(time, [d.x for d in data])
        axes_x.set_ylabel("X\n(m)")
        axes_x.grid(True)

        if m := re.search(r"(\d+)_(\d+)_(\d+)_to_(\d+)_(\d+)_(\d+)_3d", name):
            title = "\n".join(
                [
                    f"start: {m[1]}/{m[2]}/{m[3]}",
                    f"goal: {m[4]}/{m[5]}/{m[6]}",
                    "3D path",
                ]
            )
        elif m := re.search(r"(\d+)_(\d+)_(\d+)_to_(\d+)_(\d+)_(\d+)_5d_segment_(\d+)", name):
            title = "\n".join(
                [
                    f"start: {m[1]}/{m[2]}/{m[3]}",
                    f"goal: {m[4]}/{m[5]}/{m[6]}",
                    "3D path",
                    f"5D segment #{m[7]}",
                    f"Estimated wheel distance: {wheel_distance:.3f} m",
                ]
            )
        axes_x.set_title(title)

        plot += 1
        axes_y = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
        axes_y.plot(time, [d.y for d in data])
        axes_y.set_ylabel("Y\n(m)")
        axes_y.grid(True)

        plot += 1
        axes_t = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
        axes_t.plot(time, [d.theta for d in data])
        axes_t.set_ylabel("$\\theta$\n(rad)")
        axes_t.grid(True)

        if five:
            plot += 1
            axes_l = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
            axes_l.plot(time, [d.linear_index for d in data])
            axes_l.set_ylabel("linear velocity (index/value)")
            axes_l.yaxis.set_major_formatter(
                matplotlib.ticker.FuncFormatter(
                    functools.partial(_linear_labels, linear_velocity_lut)
                )
            )
            axes_l.grid(True)

            plot += 1
            axes_a = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
            axes_a.plot(time, [d.angular_index for d in data])
            axes_a.set_ylabel("angular velocity (index/value)")
            axes_a.yaxis.set_major_formatter(
                matplotlib.ticker.FuncFormatter(
                    functools.partial(_angular_labels, angular_velocity_lut)
                )
            )
            axes_a.grid(True)

            plot += 1
            axes_w = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
            axes_w.plot(time, left_wheel_speeds, color="b")
            axes_w.plot(time, right_wheel_speeds, color="r")
            axes_w.set_ylabel("wheel speeds (m/s)\n(blue left, red right)")
            axes_w.grid(True)

        # Costs and heuristic values on one plot.
        plot += 1
        axes_c = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
        axes_c.scatter(time, [d.heuristic for d in data], marker="x")
        axes_c.scatter(time, [d.cost for d in data], marker="+", color="r")
        axes_c.set_ylabel("heuristic (blue)\npath cost (red)")
        axes_c.set_xlabel("time (sec)" if five else "pose index")
        axes_c.grid(True)

        matplotlib.pyplot.setp(axes_x.get_xticklabels(), visible=False)
        matplotlib.pyplot.setp(axes_y.get_xticklabels(), visible=False)
        matplotlib.pyplot.setp(axes_t.get_xticklabels(), visible=False)

        filepath = str((self._output_directory / f"plots-{name}").with_suffix(".png"))
        fig.savefig(filepath, bbox_inches="tight")
        self.get_logger().info(f"Plot saved to '{filepath}'.")

    def plot_motion_model(
        self, data, name, motion_model, linear_velocity_lut, angular_velocity_lut
    ):
        """Create a plot that shows the velocity combinations of the motion model
        and how often they were used along the path."""

        if not data:
            self.get_logger().info(f"'data' is empty, not plotting motion model.")
            return

        # Determine if this path moves forward or backward, by looking at the
        # linear velocity index of the first pose.
        forward = data[0].linear_index > 0

        # Count the index combinations that were actually used.
        index_counter = collections.Counter(
            (d.linear_index, d.angular_index) for d in data
        )

        # The number of times the most common combination was used.
        most_used = index_counter.most_common(1)[0][1]

        # A list containing all linear/angular velocity index
        # combinations of the motion model.
        index_combinations = list(
            zip(motion_model.linear_indices, motion_model.angular_indices)
        )

        # Filter out the combinations for the one side we are interrested in for this path.
        if forward:
            index_combinations = [(l, a) for l, a in index_combinations if l > 0]
        else:
            index_combinations = [(l, a) for l, a in index_combinations if l < 0]

        # Sizes and colors for the items in 'index_combinations'.
        sizes = []
        colors = []

        for linear_index, angular_index in index_combinations:
            # The number of times this combination was used.
            uses = index_counter.get((linear_index, angular_index), 0)

            size = 20 * 2 ** (5 * uses / most_used)
            sizes.append(size)

            colors.append("r" if uses == 0 else "b")

        linear_indices, angular_indices = zip(*index_combinations)

        fig = matplotlib.figure.Figure(figsize=(HEIGHT / DPI, HEIGHT / DPI), dpi=DPI)
        fig.suptitle("Velocity Index Pairs", fontsize=20)

        axes = fig.gca()
        axes.scatter(linear_indices, angular_indices, sizes, colors)
        axes.set_xlabel("Linear Velocity Index")
        axes.set_ylabel("Angular Velocity Index")
        axes.set_aspect("equal", adjustable="box")
        axes.set_xticks(range(min(linear_indices), max(linear_indices) + 1))
        axes.set_yticks(range(min(angular_indices), max(angular_indices) + 1))
        axes.xaxis.set_major_formatter(
            matplotlib.ticker.FuncFormatter(
                functools.partial(_linear_labels, linear_velocity_lut)
            )
        )
        axes.yaxis.set_major_formatter(
            matplotlib.ticker.FuncFormatter(
                functools.partial(_angular_labels, angular_velocity_lut)
            )
        )
        axes.grid(True)

        if m := re.search(r"(\d+)_(\d+)_(\d+)_to_(\d+)_(\d+)_(\d+)_5d_segment_(\d+)", name):
            axes.set_title(
                f"start: {m[1]}/{m[2]}/{m[3]}\ngoal: {m[4]}/{m[5]}/{m[6]}\n"
                f"5D segment #{m[7]}\nmost common: {most_used}"
            )

        filepath = str((self._output_directory / f"mm-{name}").with_suffix(".png"))
        fig.savefig(filepath, bbox_inches="tight")
        self.get_logger().info(f"Plot saved to '{filepath}'.")


def main(args=None):
    rclpy.init(args=args)

    plot_augmented_path = PlotAugmentedPath(
        "/planner_server/trajectory_planner/augmented_path",
        "/global_costmap/global_costmap",
        pathlib.Path.cwd(),
    )

    rclpy.spin(plot_augmented_path)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
