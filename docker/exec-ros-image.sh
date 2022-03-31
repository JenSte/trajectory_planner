#!/bin/bash

# Start an additional shell inside the container. (Use
# 'docker/run-ros-image.sh' to start the container first.)

set -eu

CONTAINER=trajectory-planner-container

if command -v podman &> /dev/null; then
    DOCKER=podman
else
    DOCKER=docker
fi

$DOCKER exec \
    -it \
    "$CONTAINER" \
    /entrypoint.sh bash
