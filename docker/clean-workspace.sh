#!/bin/bash

# Remove build artifacts from the workspace folder.

set -eu

SCRIPT_PATH=$(dirname "$0")

rm -rf \
    "$SCRIPT_PATH/build" \
    "$SCRIPT_PATH/install" \
    "$SCRIPT_PATH/log"
