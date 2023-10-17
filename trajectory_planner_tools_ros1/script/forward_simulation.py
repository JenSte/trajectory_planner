#!/usr/bin/env python3

# Script that does a simulation of the path planned by the trajectory planner.

import dataclasses
import itertools
import logging
import math

import rospy
import transforms3d
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from trajectory_planner_msgs.msg import AugmentedPath

@dataclasses.dataclass
class SimulationDataPoint:
    """Data for one simulation step."""

    # The duration this data is valid, in seconds.
    delta_time: float

    # The linear velocity to simulate.
    linear_velocity: float

    # The angular velocity to simulate.
    angular_velocity: float


@dataclasses.dataclass
class RobotPose:
    time: float
    x: float
    y: float
    theta: float


class ForwardSimulation:
    """Take the trajectory_planner's augmented path and simulate driving it."""

    def __init__(
        self, simulation_time_delta, honor_wheel_limits, augmented_path_topic, simulated_path_topic
    ):

        self._simulation_time_delta = simulation_time_delta
        self._honor_wheel_limits = honor_wheel_limits

        # Used to collect all the segments that belong to a 5D search.
        self._5d_segments = []

        self._sub_augmented_path = rospy.Subscriber(
            augmented_path_topic,
            AugmentedPath,
            self._augmented_path_callback,
        )

        self._pub_simulated_path = rospy.Publisher(
            simulated_path_topic,
            Path,
            queue_size=10,
        )

    def _augmented_path_callback(self, message):
        rospy.loginfo(f"Received path '{message.name}'.")

        if "_5d_" not in message.name:
            # Only 5D paths can be simulated.
            return

        if message.segment_count == 0:
            self._5d_segments = []

        self._5d_segments.append(message)

        if message.segment_count == (message.total_segments - 1):
            rospy.loginfo(f"Collected {len(self._5d_segments)} 5D segments, simulating...")

            simulated_paths = self._simulate_path(self._5d_segments)

            # Create a ROS path message by combining the simulated segments.
            msg = Path()
            msg.header.frame_id = "map"

            for simulated_segment in simulated_paths:
                msg.poses.extend(
                    self._stamped_poses_from_simulated_path(
                        simulated_segment, 0.0, msg.header.frame_id,
                    )
                )

            self._pub_simulated_path.publish(msg)

    def _stamped_poses_from_simulated_path(self, simulated_path, start_time, frame_id):
        """Return a list of ROS PoseStamped messages."""

        result = []
        for pose in simulated_path:
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.from_sec(start_time + pose.time)
            msg.header.frame_id = frame_id

            msg.pose.position.x = pose.x
            msg.pose.position.y = pose.y
            msg.pose.position.z = 0.0

            q = transforms3d.euler.euler2quat(0, 0, pose.theta)
            msg.pose.orientation.w = q[0]
            msg.pose.orientation.x = q[1]
            msg.pose.orientation.y = q[2]
            msg.pose.orientation.z = q[3]

            result.append(msg)

        return result

    def _simulate_path(self, segments):
        """Do a forward simulation of the path using the collected 5D segments."""

        simulated_paths = []
        for i, segment in enumerate(segments):
            if segment.segment_type == AugmentedPath.SEGMENT_TYPE_TURN:
                # TODO: Apply the change in the robot's orientation to the
                # last pose of the simulated path, and append the new pose.
                rospy.loginfo(f"Segment {i}: Turn segment, skipping.")
            else:
                # This is a forward/backward driving segment.
                if not segment.poses:
                    rospy.logwarn(f"Segment {i}: Empty augmented path, skipping.")
                    continue

                simulated_paths.append(self._simulate_drive_segment(segment))

        return simulated_paths

    def _extract_yaw(self, quaternion):
        """Extract the robot's orientation angle from a ROS quaternion message."""

        angles = transforms3d.euler.quat2euler(
            (quaternion.w, quaternion.x, quaternion.y, quaternion.z)
        )

        return angles[2] % (2 * math.pi)

    def _wheel(self, limited_list, velocity_limit, acceleration_max, start_velocity=0.0):
        """A generator that simulates a single wheel."""

        # Add a percent to the limit, so that the comparison below does not
        # lead to false positives.
        velocity_limit *= 1.01

        # The current velocity of the wheel.
        velocity = start_velocity

        while True:
            # The maximum and minimum velocities that are allowed
            # by the given acceleration limit.
            velocity_max = velocity + acceleration_max * self._simulation_time_delta
            velocity_min = velocity - acceleration_max * self._simulation_time_delta

            # The velocity "target" is the "input" to the simulated wheel.
            target_velocity = yield velocity

            if target_velocity > velocity:
                velocity = min(target_velocity, velocity_max)
            else:
                velocity = max(target_velocity, velocity_min)

            # If the min()/max() above returned the limit instead of 'target_velocity',
            # it means that the change in velocity was limited by the acceleration limit.
            limited_list.append(not math.isclose(velocity, target_velocity))

            if not (-velocity_limit < velocity < velocity_limit):
                # If the path is constructed correctly, with a motion model that takes
                # this limit into account, the velocity should never be above this limit.
                rospy.logwarn(
                    f"Wheel is too fast/too slow ({velocity:.2f} m/s, limit = {velocity_limit:.2f} m/s)."
                )

    def _simulate_drive_segment(self, segment):
        """Do a forward simulation of the given segment."""

        rospy.logdebug("Simulating a drive segment, motion model:")
        rospy.logdebug(f"    maximum wheel velocity: {segment.motion_model.maximum_wheel_velocity:.2f} m/s")
        rospy.logdebug(f"    maximum wheel acceleration: {segment.motion_model.maximum_wheel_acceleration:.2f} m/(s^2)")
        rospy.logdebug(f"    wheel distance: {segment.motion_model.wheel_distance:.2f} m")

        # Create lookup tables to convert the linear/angular velocity indices
        # to actual values.
        linear_velocity_lut = {}
        for i, v in enumerate(segment.motion_model.linear_velocities):
            linear_velocity_lut[i] = v
            if i != 0:
                linear_velocity_lut[-i] = -v

        angular_velocity_lut = {}
        for i, v in enumerate(segment.motion_model.angular_velocities):
            angular_velocity_lut[i] = v
            if i != 0:
                angular_velocity_lut[-i] = -v

        # The input data for the simulation. This is the time between each two
        # poses, and the velocities to drive between the two poses.
        simdata = []
        for i in range(0, len(segment.poses) - 1):
            # The timestamps of the two poses.
            time_a = segment.poses[i].pose.header.stamp.to_time()
            time_b = segment.poses[i + 1].pose.header.stamp.to_time()

            simdata.append(
                SimulationDataPoint(
                    time_b - time_a,
                    linear_velocity_lut[segment.poses[i].linear_index],
                    angular_velocity_lut[segment.poses[i].angular_index],
                )
            )

        # Create two "wheels" for the simuation
        if self._honor_wheel_limits:
            rospy.logdebug("Using the wheel limits from the motion model.")
            maximum_wheel_velocity = segment.motion_model.maximum_wheel_velocity
            maximum_wheel_acceleration = segment.motion_model.maximum_wheel_acceleration
        else:
            rospy.logdebug("Not using the wheel limits from the motion model.")
            maximum_wheel_velocity = 1e6
            maximum_wheel_acceleration = 1e6
        left_wheel_limited = []
        right_wheel_limited = []
        left_wheel = self._wheel(left_wheel_limited, maximum_wheel_velocity, maximum_wheel_acceleration)
        right_wheel = self._wheel(right_wheel_limited, maximum_wheel_velocity, maximum_wheel_acceleration)

        # Advance the iterator to their first "yield".
        left_wheel.send(None)
        right_wheel.send(None)

        start_pose = RobotPose(
            0.0,
            segment.poses[0].pose.pose.position.x,
            segment.poses[0].pose.pose.position.y,
            self._extract_yaw(segment.poses[0].pose.pose.orientation),
        )

        simulated_path = [start_pose]
        for s in simdata:
            # The speeds requested by the data point if the wheels would be able to go indefinitely fast.
            left_wheel_target_velocity = s.linear_velocity - segment.motion_model.wheel_distance * s.angular_velocity / 2.0
            right_wheel_target_velocity = s.linear_velocity + segment.motion_model.wheel_distance * s.angular_velocity / 2.0

            for i in range(round(s.delta_time / self._simulation_time_delta)):
                # The wheel speeds limited by the simulated wheels.
                left_wheel_actual_velocity = left_wheel.send(left_wheel_target_velocity)
                right_wheel_actual_velocity = right_wheel.send(right_wheel_target_velocity)

                # The linear/angular velocities with which the robot actually drives.
                actual_linear_velocity = (right_wheel_actual_velocity + left_wheel_actual_velocity) / 2.0
                actual_angular_velocity = (right_wheel_actual_velocity - left_wheel_actual_velocity) / segment.motion_model.wheel_distance

                last_pose = simulated_path[-1]

                dx = actual_linear_velocity * math.cos(last_pose.theta) * self._simulation_time_delta
                dy = actual_linear_velocity * math.sin(last_pose.theta) * self._simulation_time_delta
                dtheta = actual_angular_velocity * self._simulation_time_delta

                simulated_path.append(
                    RobotPose(
                        len(simulated_path) * self._simulation_time_delta,
                        last_pose.x + dx,
                        last_pose.y + dy,
                        (last_pose.theta + dtheta) % (2 * math.pi),
                    )
                )

        left_limitations = sum(1 for l in left_wheel_limited if l)
        left_percentage = 100.0 * left_limitations / len(left_wheel_limited)
        right_limitations = sum(1 for r in right_wheel_limited if r)
        right_percentage = 100.0 * right_limitations / len(right_wheel_limited)

        rospy.loginfo(
            f"Left wheel was limited in {left_limitations} of {len(left_wheel_limited)} "
            f"simulation steps ({left_percentage:.2f} %)."
        )
        rospy.loginfo(
            f"Right wheel was limited in {right_limitations} of {len(right_wheel_limited)} "
            f"simulation steps ({right_percentage:.2f} %)."
        )

        return simulated_path


if __name__ == "__main__":
    try:
        rospy.init_node("forward_simulation")

        logging.getLogger("rosout").setLevel(logging.DEBUG)

        # Time step for the simulation of the path.
        delta_time = rospy.get_param("~delta_time", 1.0/1000.0)

        # Wheter or not to use the velocity/acceleration limits from the
        # motion model in the augmented path message.
        honor_wheel_limits = rospy.get_param("~honor_wheel_limits", True)

        forward_simulation = ForwardSimulation(
            delta_time, honor_wheel_limits, "~path", "~simulated_path"
        )

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
