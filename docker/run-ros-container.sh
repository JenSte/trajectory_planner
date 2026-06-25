#!/bin/bash

# Run the Docker image, and drop to the shell by default. Pass
# an additional command to this script to run it instead.

set -eu

SCRIPT_PATH=$(dirname `realpath "$0"`)
source "$SCRIPT_PATH/config"

if command -v podman &> /dev/null; then
    DOCKER=podman
else
    DOCKER=docker
fi

# Create a variable that contains the arguments to mount the correct source
# directories into the container. The list of volumes is initialized with
# directories that are used in both ROS1 and ROS2 plugins.
V=""
V="$V --volume $SCRIPT_PATH/../map:/opt/map"
V="$V --volume $SCRIPT_PATH/../trajectory_planner:/opt/trajectory_planner"
V="$V --volume $SCRIPT_PATH/../trajectory_planner_plugin:/opt/trajectory_planner_plugin"

# Unfortunately, the catkin-tools ignore packages that container a file named
# "COLCON_IGNORE", so it's not as easy as just putting "CATKIN_IGNORE" files
# into ROS2 and "COLCON_IGNORE" files into ROS1 packages.
if [ "$ROS_VERSION" = "melodic" ] || [ "$ROS_VERSION" = "noetic" ]; then
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_msgs_ros1:/opt/trajectory_planner_msgs_ros1"
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_plugin_ros1:/opt/trajectory_planner_plugin_ros1"
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_tools_ros1:/opt/trajectory_planner_tools_ros1"

    # Package containing models used for the simulation, located at
    # https://github.com/tuw-robotics/tuw_gazebo
    if [ -d "$SCRIPT_PATH/tuw_gazebo" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_gazebo:/opt/tuw_gazebo"
    fi
    # https://github.com/tuw-robotics/tuw_msgs.git
    if [ -d "$SCRIPT_PATH/tuw_msgs" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_msgs:/opt/tuw_msgs"
    fi
    # https://github.com/tuw-robotics/tuw_global_planner.git
    if [ -d "$SCRIPT_PATH/tuw_global_planner" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_global_planner:/opt/tuw_global_planner"
    fi
    # git@github.com:tuw-robotics/tuw_launches.git
    if [ -d "$SCRIPT_PATH/tuw_launches" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_launches:/opt/tuw_launches"
    fi
    # git@github.com:tuw-robotics/tuw_rviz.git
    if [ -d "$SCRIPT_PATH/tuw_rviz" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_rviz:/opt/tuw_rviz"
    fi
    # git@github.com:tuw-robotics/tuw_multi_robot.git
    if [ -d "$SCRIPT_PATH/tuw_multi_robot" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_multi_robot:/opt/tuw_multi_robot"
    fi
    # git@github.com:tuw-robotics/tuw_geometry.git
    if [ -d "$SCRIPT_PATH/tuw_geometry" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_geometry:/opt/tuw_geometry"
    fi
    # git@github.com:tuw-robotics/tuw_diffdrive.git
    if [ -d "$SCRIPT_PATH/tuw_diffdrive" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_diffdrive:/opt/tuw_diffdrive"
    fi
    # git@github.com:tuw-robotics/tuw_control.git
    if [ -d "$SCRIPT_PATH/tuw_control" ]; then
        V="$V --volume $SCRIPT_PATH/tuw_control:/opt/tuw_control"
    fi
else
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_msgs_ros2:/opt/trajectory_planner_msgs_ros2"
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_plugin_ros2:/opt/trajectory_planner_plugin_ros2"
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_tools_ros2:/opt/trajectory_planner_tools_ros2"

    # Package containing models used for the simulation, located at
    # https://github.com/tuw-robotics/tuw2_gazebo
    if [ -d "$SCRIPT_PATH/tuw2_gazebo" ]; then
        V="$V --volume $SCRIPT_PATH/tuw2_gazebo:/opt/tuw2_gazebo"
    fi
    # https://github.com/tuw-robotics/Stage.git
    if [ -d "$SCRIPT_PATH/Stage" ]; then
        V="$V --volume $SCRIPT_PATH/Stage:/opt/Stage"
    fi
    # https://github.com/tuw-robotics/stage_ros2.git
    if [ -d "$SCRIPT_PATH/stage_ros2" ]; then
        V="$V --volume $SCRIPT_PATH/stage_ros2:/opt/stage_ros2"
    fi
fi

$DOCKER run \
    -it --rm \
    --name "$CONTAINER" \
    --hostname ros \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume $XAUTHORITY:/root/.Xauthority \
    --volume "$SCRIPT_PATH/workspace:/opt/ws" \
    $V \
    "$TAG" "$@"
