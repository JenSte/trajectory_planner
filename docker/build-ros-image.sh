#!/bin/bash

# Build a Docker image with a ROS2 installation in it.

set -eu

# The ROS2 version to install.
ROS_VERSION=galactic
#ROS_VERSION=humble

# The base OS image.
if [ "$ROS_VERSION" = "galactic" ]; then
    BASE_IMAGE=ubuntu:focal
else
    BASE_IMAGE=ubuntu:jammy
fi

# The tag of the image that is created.
TAG=trajectory-planner-build-image

if command -v podman &> /dev/null; then
    DOCKER=podman
else
    DOCKER=docker
fi

SCRIPT_PATH=$(dirname `realpath "$0"`)

# Update base image.
$DOCKER pull $BASE_IMAGE

# Create backup of old image.
$DOCKER image tag "$TAG" "$TAG:old" || true

# Build the new image.
$DOCKER build \
    --tag "$TAG" \
    --file "$SCRIPT_PATH/Dockerfile" \
    --build-arg "ROS_VERSION=$ROS_VERSION" \
    --build-arg "BASE_IMAGE=$BASE_IMAGE" \
    "$SCRIPT_PATH"

# Remove the backup image.
$DOCKER image rm "$TAG:old" || true
