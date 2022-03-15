#!/bin/bash

# Run the Docker image, and drop to the shell by default. Pass
# an additional command to this script to run it instead.

set -eu

TAG=trajectory-planner-build-image

SCRIPT_PATH=$(dirname "$0")

podman run \
    -it --rm \
    --hostname ros \
    --env DISPLAY=$DISPLAY \
    --volume /tmp/.X11-unix:/tmp/.X11-unix \
    --volume "$SCRIPT_PATH/workspace:/opt/ws" \
    --volume "$SCRIPT_PATH/..:/opt/trajectory_planner" \
    "$TAG" "$@"
