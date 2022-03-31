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

SCRIPT_PATH=$(dirname "$0")

$DOCKER run \
    -it --rm \
    --name "$CONTAINER" \
    --hostname ros \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume "$SCRIPT_PATH/workspace:/opt/ws" \
    --volume "$SCRIPT_PATH/..:/opt/trajectory_planner" \
    "$TAG" "$@"
