#!/bin/bash

# Built the stand-alone example, and show it's profiling data.

set -eu

SCRIPT_PATH=$(readlink -f `dirname "$0"`)

mkdir -p build-standalone
cd build-standalone

cmake "$SCRIPT_PATH"
make -j4

perf record --call-graph dwarf -- ./trajectory_planner_standalone
hotspot perf.data
