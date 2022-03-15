#!/bin/bash

# Build the workspace.

set -eu

SCRIPT_PATH=$(dirname "$0")

"$SCRIPT_PATH/run-ros-container.sh" \
    "colcon build --allow-overriding multidimension_planner"
