#!/usr/bin/env python3

# This script takes a CSV file created by the TrajectoryPlanner's 'write_result_debug_data()'
# function and creates a plot from the data within.

import argparse
import collections
import csv
import pathlib
import sys

import matplotlib
import matplotlib.figure
import matplotlib.pyplot
import matplotlib.ticker
matplotlib.use("agg")

DataPoint = collections.namedtuple(
    "DataPoint", [
        "time",
        "x",
        "y",
        "theta",
        "linear_velocity_index",
        "linear_velocity",
        "angular_velocity_index",
        "angular_velocity",
        "cost",
        "heuristic",
    ]
)


def read_data(input_file):
    """Read the path data from a CSV file."""

    result = []

    with open(input_file) as file:
        reader = csv.DictReader(file)
        for row in reader:
            result.append(
                DataPoint(
                    float(row.get("time", "0.0")),
                    int(row.get("x")),
                    int(row.get("y")),
                    float(row.get("theta")),
                    int(row.get("linear_velocity_index", "0")),
                    float(row.get("linear_velocity", "0.0")),
                    int(row.get("angular_velocity_index", "0")),
                    float(row.get("angular_velocity", "0.0")),
                    float(row.get("cost")),
                    float(row.get("heuristic")),
                )
            )

    return result


def plot_data(data, output_file, width, height, dpi, wheel_distance):
    """Plot the data and store the plot to a file."""

    # Determine if we plot a five dimensional path by looking at the name.
    five = "_5d_" in output_file.name

    if five:
        time = [d.time for d in data]
        subplots = 8
    else:
        # 3D paths do not include timestamps for the poses.
        time = range(len(data))
        subplots = 4

    if five:
        # In 5D mode, when we have vehicle velocities available, calculate the wheel speeds.
        right_wheel_speeds = []
        left_wheel_speeds = []
        right_wheel_accelerations = []
        left_wheel_accelerations = []

        for i, d in enumerate(data):
            left_wheel_speeds.append(d.linear_velocity - wheel_distance * d.angular_velocity / 2.0)
            right_wheel_speeds.append(d.linear_velocity + wheel_distance * d.angular_velocity / 2.0)

            if i > 0:
                dt = d.time - data[i - 1].time
                right_wheel_accelerations.append((right_wheel_speeds[-1] - right_wheel_speeds[-2]) / dt)
                left_wheel_accelerations.append((left_wheel_speeds[-1] - left_wheel_speeds[-2]) / dt)

    plot = 0

    fig = matplotlib.figure.Figure(figsize=(width / dpi, height / dpi), dpi=dpi)
    fig.suptitle("Path Details", fontsize=20)

    # x
    plot += 1
    axes_x = fig.add_subplot(subplots, 1, plot)
    axes_x.plot(time, [d.x for d in data])
    axes_x.set_ylabel("X\n(m)")
    axes_x.grid(True)

    # y
    plot += 1
    axes_y = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
    axes_y.plot(time, [d.y for d in data])
    axes_y.set_ylabel("Y\n(m)")
    axes_y.grid(True)

    # theta
    plot += 1
    axes_t = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
    axes_t.plot(time, [d.theta for d in data])
    axes_t.set_ylabel("$\\theta$\n(rad)")
    axes_t.grid(True)

    if five:
        plot += 1
        axes_l = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
        axes_l.plot(time, [d.linear_velocity_index for d in data])
        axes_l.set_ylabel("linear velocity index")
        axes_l.grid(True)

        plot += 1
        axes_a = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
        axes_a.plot(time, [d.angular_velocity_index for d in data])
        axes_a.set_ylabel("angular velocity index")
        axes_a.grid(True)

        plot += 1
        axes_w = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
        axes_w.plot(time, left_wheel_speeds, color="b")
        axes_w.plot(time, right_wheel_speeds, color="r")
        axes_w.set_ylabel("wheel speeds (m/s)\n(blue left, red right)")
        axes_w.grid(True)

        plot += 1
        axes_b = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
        axes_b.plot(time[:-1], left_wheel_accelerations, color="b")
        axes_b.plot(time[:-1], right_wheel_accelerations, color="r")
        axes_b.set_ylabel("wheel accelerations (m/s^2)\n(blue left, red right)")
        axes_b.grid(True)

    # Costs and heuristic values on one plot.
    plot += 1
    axes_c = fig.add_subplot(subplots, 1, plot, sharex=axes_x)
    axes_c.scatter(time, [d.heuristic for d in data], marker="x")
    axes_c.scatter(time, [d.cost for d in data], marker="+", color="r")
    axes_c.set_ylabel("heuristic (blue)\npath cost (red)")
    axes_c.set_xlabel("time (sec)" if five else "pose index")
    axes_c.grid(True)

    # If the heuristic overestimates the cost of a pose on the path, we color
    # the background of the cost/heuristic plot.
    overestimates = [d.heuristic > d.cost for d in data]
    if any(overestimates):
        axes_c.set_facecolor("yellow")

        for d, t in zip(data, time):
            if d.heuristic > d.cost:
                axes_c.axvspan(t - 0.1, t + 0.1, facecolor="r", alpha=0.5)

                print(
                    f"bad heuristic value at path element {t}, h = {d.heuristic:.2f}, c = {d.cost:.2f}"
                )

    matplotlib.pyplot.setp(axes_x.get_xticklabels(), visible=False)
    matplotlib.pyplot.setp(axes_y.get_xticklabels(), visible=False)
    matplotlib.pyplot.setp(axes_t.get_xticklabels(), visible=False)

    fig.savefig(output_file, bbox_inches="tight")
    print(f"Plot saved to '{output_file}'.")


def main():
    parser = argparse.ArgumentParser(
        description="Plot paths created by the TrajectoryPlanner.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--wheel-distance",
        default=1.0,
        help="Vehicles distance between the wheels, in meters.",
    )
    parser.add_argument(
        "--plot-width",
        default=2400,
        help="Width of the created plot.",
    )
    parser.add_argument(
        "--plot-height",
        default=2000,
        help="Height of the created plot.",
    )
    parser.add_argument(
        "--plot-dpi",
        default=100,
        help="DPI number of the created plot.",
    )
    parser.add_argument(
        "input",
        help="The CSV file to process.",
    )
    args = parser.parse_args()

    input_file = pathlib.Path(args.input)
    output_file = input_file.with_suffix(".png")

    plot_data(
        read_data(input_file),
        output_file,
        args.plot_width,
        args.plot_height,
        args.plot_dpi,
        args.wheel_distance,
    )


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        sys.exit(str(e))
