#!/bin/bash

# Remove build artifacts from the workspace folder.

set -eu

SCRIPT_PATH=$(dirname `realpath "$0"`)

rm -rf \
    "$SCRIPT_PATH/workspace/.catkin_workspace" \
    "$SCRIPT_PATH/workspace/CMakeLists.txt" \
    "$SCRIPT_PATH/workspace/build" \
    "$SCRIPT_PATH/workspace/devel" \
    "$SCRIPT_PATH/workspace/install" \
    "$SCRIPT_PATH/workspace/log"
