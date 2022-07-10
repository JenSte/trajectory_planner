#!/usr/bin/env python3

# This script listens for an augmented path message and then does the forward
# simulation again, using two methods:
#  - without the quantization error to map cell coordinates and discrete angle values
#  - with a simulated differential drive that limits the acceleration of the wheels
# The result of both simulations is published as path messages to allow comparison
# between the path the global planner generates and the simulated ones.

import collections
import math

import rclpy.node
import rclpy.qos
import transforms3d

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from trajectory_planner.msg import AugmentedPath

import plot_augmented_path
import utils


SimulationDataPoint = collections.namedtuple(
    "SimulationDataPoint", [
        "delta_time",
        "linear_velocity",
        "angular_velocity",
    ]
)

RobotPose = collections.namedtuple(
    "RobotPose", [
        "time",
        "x",
        "y",
        "theta",
    ]
)


class ForwardSimulation(rclpy.node.Node):

    def __init__(
        self, augmented_path_topic, costmap_node, unquanizised_path_topic, differential_drive_path_topic
    ):
        super().__init__("forward_simulation")

        footprint = utils.get_footprint(self, costmap_node)
        self._wheel_distance = utils.estimate_wheel_distance(footprint)
        self.get_logger().info(f"Estimated wheel distance: {self._wheel_distance:.3f} m")

        self._sub_augmented_path = self.create_subscription(
            AugmentedPath,
            augmented_path_topic,
            self.augmented_path_callback,
            10,
        )

        self._pub_unquantisized_path = self.create_publisher(
            Path,
            unquanizised_path_topic,
            rclpy.qos.qos_profile_system_default,
        )

        self._pub_differential_drive_path = self.create_publisher(
            Path,
            differential_drive_path_topic,
            rclpy.qos.qos_profile_system_default,
        )

        self._wheel_velocity_max = 1.0
        self._wheel_acceleration_max = 1.5

    def augmented_path_callback(self, augmented_path):
        """Callback for the augmented path subscriber."""

        if "_5d_" not in augmented_path.name:
            # We only look at 5D paths.
            return

        if not augmented_path.poses:
            self.get_logger().info("Received empty augmented path message.")
            return

        linear_velocity_lut, angular_velocity_lut = plot_augmented_path.create_luts(
            augmented_path.motion_model
        )

        # The poses from the augmented path.
        data = [
            plot_augmented_path.extract_data_point(pose) for pose in augmented_path.poses
        ]

        # The start pose of the simulation.
        start = RobotPose(0.0, data[0].x, data[0].y, data[0].theta)

        # The information needed to do the simulation.
        simdata = []
        for i, pose in enumerate(data):
            # The time we simulate between these two poses.
            if i < len(data) - 1:
                # Use the delta time to the next timestamp.
                dt = data[i + 1].time - pose.time
            else:
                # The last pose on the path, re-use the last delta time.
                dt = simdata[-1].delta_time

            linear_velocity = linear_velocity_lut[pose.linear_index]
            angular_velocity = angular_velocity_lut[pose.angular_index]

            simdata.append(SimulationDataPoint(dt, linear_velocity, angular_velocity))

        unquantisized_path = self.unquantisized_simulation(start, simdata)
        differential_drive_path = self.differential_drive_simulation(
            start,
            simdata,
            self._wheel_distance,
            self._wheel_velocity_max,
            self._wheel_acceleration_max,
        )

        self.log_error(data[-1], unquantisized_path[-1], "unquantisized path")
        self.log_error(data[-1], differential_drive_path[-1], "differential drive path")

        self._pub_unquantisized_path.publish(
            self.create_path_message(unquantisized_path)
        )
        self._pub_differential_drive_path.publish(
            self.create_path_message(differential_drive_path)
        )

    def log_error(self, goal_5d, goal_simulated, s):
        """Calculate the error between the original pose from the global planner
        and a pose from the simulation."""

        # Calculate the distance error between the two poses.
        dx = goal_5d.x - goal_simulated.x
        dy = goal_5d.y - goal_simulated.y
        distance_error = math.sqrt(math.pow(dx, 2.0) + math.pow(dy, 2.0))

        # Calculate the orientation error between the two poses.
        orientation_error = math.atan2(
            math.sin(goal_5d.theta - goal_simulated.theta),
            math.cos(goal_5d.theta - goal_simulated.theta),
        )
        orientation_error = math.degrees(abs(orientation_error))

        orientation_5d = math.degrees(goal_5d.theta)
        orientation_simulated = math.degrees(goal_simulated.theta)

        self.get_logger().info(f"Error between original path and {s}:")
        self.get_logger().info(f"  original goal:     {goal_5d.x:.3f} m / {goal_5d.y:.3f} m / {orientation_5d:.3f} deg")
        self.get_logger().info(f"  simulated goal:    {goal_simulated.x:.3f} m / {goal_simulated.y:.3f} m / {orientation_simulated:.3f} deg")
        self.get_logger().info(f"  distance error:    {distance_error:.3f} m")
        self.get_logger().info(f"  orientation error: {orientation_error:.3f} deg")

    def unquantisized_simulation(self, start, simdata):
        """Simulate a path, without rounding the coordinates to map cells."""

        simulated_path = [start]
        for dt, linear_velocity, angular_velocity in simdata:
            last_pose = simulated_path[-1]

            dx = linear_velocity * math.cos(last_pose.theta) * dt
            dy = linear_velocity * math.sin(last_pose.theta) * dt
            dtheta = angular_velocity * dt

            new_pose = RobotPose(
                last_pose.time + dt,
                last_pose.x + dx,
                last_pose.y + dy,
                (last_pose.theta + dtheta) % (2 * math.pi),
            )

            simulated_path.append(new_pose)

        return simulated_path

    def differential_drive_simulation(
        self, start, simdata, wheel_distance, wheel_velocity_max, wheel_acceleration_max
    ):
        """Simulate a path, by simulating a differential drive."""

        # The time difference between simulation steps.
        simulation_time_delta = 0.005

        def wheel(velocity_max, acceleration_max, velocity=0.0):
            """A generator that simulates a single wheel."""

            while True:
                # The maximum and minimum velocities that are allowed
                # by the given acceleration limit.
                velocity_max = velocity + acceleration_max * simulation_time_delta
                velocity_min = velocity - acceleration_max * simulation_time_delta

                # The velocity "target" is the "input" to the simulated wheel.
                target_velocity = yield velocity

                if target_velocity > velocity:
                    velocity = min(target_velocity, velocity_max)
                else:
                    velocity = max(target_velocity, velocity_min)

        left_wheel = wheel(wheel_velocity_max, wheel_acceleration_max)
        right_wheel = wheel(wheel_velocity_max, wheel_acceleration_max)
        left_wheel.send(None)
        right_wheel.send(None)

        simulated_path = [start]
        simulated_steps = 1  # The start pose is al
        for time_delta, linear_velocity, angular_velocity in simdata:
            # The speed we want to go to when simulating this data point.
            left_wheel_target_velocity = linear_velocity - wheel_distance * angular_velocity / 2.0
            right_wheel_target_velocity = linear_velocity + wheel_distance * angular_velocity / 2.0

            for i in range(round(time_delta / simulation_time_delta)):
                # Simulate the acceleration of the wheels.
                left_wheel_actual_velocity = left_wheel.send(left_wheel_target_velocity)
                right_wheel_actual_velocity = right_wheel.send(right_wheel_target_velocity)

                actual_linear_velocity = (right_wheel_actual_velocity + left_wheel_actual_velocity) / 2.0
                actual_angular_velocity = (right_wheel_actual_velocity - left_wheel_actual_velocity) / wheel_distance

                last_pose = simulated_path[-1]

                dx = actual_linear_velocity * math.cos(last_pose.theta) * simulation_time_delta
                dy = actual_linear_velocity * math.sin(last_pose.theta) * simulation_time_delta
                dtheta = actual_angular_velocity * simulation_time_delta

                new_pose = RobotPose(
                    simulated_steps * simulation_time_delta,
                    last_pose.x + dx,
                    last_pose.y + dy,
                    (last_pose.theta + dtheta) % (2 * math.pi),
                )

                simulated_path.append(new_pose)
                simulated_steps += 1

        return simulated_path

    def create_path_message(self, path):
        """Create a ROS 'Path' message from a list of 'RobotPose's."""

        path_msg = Path()
        path_msg.header.frame_id = "map"
        for t, x, y, theta in path:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = "map"
            # TODO: set timestamp

            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = 0.0

            q = transforms3d.euler.euler2quat(0, 0, theta)
            pose_msg.pose.orientation.w = q[0]
            pose_msg.pose.orientation.x = q[1]
            pose_msg.pose.orientation.y = q[2]
            pose_msg.pose.orientation.z = q[3]

            path_msg.poses.append(pose_msg)

        return path_msg


def main(args=None):
    rclpy.init(args=args)

    forward_simulation = ForwardSimulation(
        "/planner_server/trajectory_planner/augmented_path",
        "/global_costmap/global_costmap",
        "~/unquantized_path",
        "~/differential_drive_path",
    )

    rclpy.spin(forward_simulation)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        pass
