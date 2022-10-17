#!/bin/bash

# Build a Docker image with a ROS installation in it.

set -eu

SCRIPT_PATH=$(dirname `realpath "$0"`)
source "$SCRIPT_PATH/config"

# Select an apropriate base OS image depending on the ROS version.
if [ "$ROS_VERSION" = "noetic" ] || [ "$ROS_VERSION" = "galactic" ]; then
    BASE_IMAGE=ubuntu:focal
else
    BASE_IMAGE=ubuntu:jammy
fi

if command -v podman &> /dev/null; then
    DOCKER=podman
else
    DOCKER=docker
fi

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
