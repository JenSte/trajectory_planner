#!/bin/bash

# Run the Docker image, and drop to the shell by default. Pass
# an additional command to this script to run it instead.

set -eu

TAG=trajectory-planner-build-image

CONTAINER=trajectory-planner-container

if command -v podman &> /dev/null; then
    DOCKER=podman
else
    DOCKER=docker
fi

SCRIPT_PATH=$(dirname `realpath "$0"`)

$DOCKER run \
    -it --rm \
    --name "$CONTAINER" \
    --hostname ros \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume "$SCRIPT_PATH/workspace:/opt/ws" \
    --volume "$SCRIPT_PATH/../trajectory_planner:/opt/trajectory_planner" \
    --volume "$SCRIPT_PATH/../trajectory_planner_msgs:/opt/trajectory_planner_msgs" \
    --volume "$SCRIPT_PATH/../trajectory_planner_tools:/opt/trajectory_planner_tools" \
    --volume "$SCRIPT_PATH/../trajectory_planner_tools_ros1:/opt/trajectory_planner_tools_ros1" \
    --volume "$SCRIPT_PATH/../trajectory_planner_plugin_ros1:/opt/trajectory_planner_plugin_ros1" \
    "$TAG" "$@"
