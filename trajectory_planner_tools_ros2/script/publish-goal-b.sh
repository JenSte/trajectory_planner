#!/bin/bash

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped 'header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  position:
    x: -5.0
    y: -6.0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 1.0
    w: 0.0
'
