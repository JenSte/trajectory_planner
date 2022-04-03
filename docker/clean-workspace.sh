#!/bin/bash

# Remove build artifacts from the workspace folder.

set -eu

SCRIPT_PATH=$(dirname "$0")

rm -rf \
    "$SCRIPT_PATH/workspace/build" \
    "$SCRIPT_PATH/workspace/install" \
    "$SCRIPT_PATH/workspace/log"
