#!/bin/bash

# Build a Docker image with a ROS2 installation in it.

set -eu

TAG=trajectory-planner-build-image

SCRIPT_PATH=$(dirname "$0")

# Update base image.
podman pull ubuntu:focal

# Create backup of old image.
podman image tag "$TAG" "$TAG:old" || true

# Build the new image.
podman build \
    --tag "$TAG" \
    --file "$SCRIPT_PATH/Dockerfile" \
    "$SCRIPT_PATH"

# Remove the backup image.
podman image rm "$TAG:old" || true
