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
if [ "$ROS_VERSION" = "noetic" ]; then
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_msgs_ros1:/opt/trajectory_planner_msgs_ros1"
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_plugin_ros1:/opt/trajectory_planner_plugin_ros1"
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_tools_ros1:/opt/trajectory_planner_tools_ros1"
else
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_msgs_ros2:/opt/trajectory_planner_msgs_ros2"
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_plugin_ros2:/opt/trajectory_planner_plugin_ros2"
    V="$V --volume $SCRIPT_PATH/../trajectory_planner_tools_ros2:/opt/trajectory_planner_tools_ros2"
fi

$DOCKER run \
    -it --rm \
    --name "$CONTAINER" \
    --hostname ros \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume "$SCRIPT_PATH/workspace:/opt/ws" \
    $V \
    "$TAG" "$@"
