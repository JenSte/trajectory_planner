#!/bin/bash

# Build a Docker image with a ROS2 installation in it.

set -eu

TAG=trajectory-planner-build-image

if command -v podman &> /dev/null; then
    DOCKER=podman
else
    DOCKER=docker
fi

SCRIPT_PATH=$(dirname "$0")

# Update base image.
$DOCKER pull ubuntu:focal

# Create backup of old image.
$DOCKER image tag "$TAG" "$TAG:old" || true

# Build the new image.
$DOCKER build \
    --tag "$TAG" \
    --file "$SCRIPT_PATH/Dockerfile" \
    "$SCRIPT_PATH"

# Remove the backup image.
$DOCKER image rm "$TAG:old" || true
