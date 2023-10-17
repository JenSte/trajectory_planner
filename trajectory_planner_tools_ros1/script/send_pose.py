#!/usr/bin/env python3

# Helper script to send a PoseStamped message.

import math

import rospy
from geometry_msgs.msg import PoseStamped
import tf.transformations


def create_pose(frame, x, y, theta):
    """Return the filled out pose message."""

    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.header.frame_id = frame

    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.position.z = 0.0

    q = tf.transformations.quaternion_from_euler(0.0, 0.0, theta)
    msg.pose.orientation.x = q[0]
    msg.pose.orientation.y = q[1]
    msg.pose.orientation.z = q[2]
    msg.pose.orientation.w = q[3]

    return msg


def main():
    rospy.init_node("send_pose", anonymous=True)
    pose_publisher = rospy.Publisher("~pose", PoseStamped, queue_size=10)

    msg = create_pose(
        rospy.get_param("~frame", "map"),
        rospy.get_param("~x", 0.0),
        rospy.get_param("~y", 0.0),
        rospy.get_param("~theta", 0.0),
    )

    # Give the publisher some time to connect.
    rospy.sleep(1.0)

    pose_publisher.publish(msg)

    # Sleep some time before shutting down to get the message out.
    rospy.sleep(1.0)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
