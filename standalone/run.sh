#!/bin/bash

# Build the stand-alone example, run it, and show it's profiling data.

set -eu

usage() { echo "Usage: $0 [-p] [-o debug-output] [-r route]" >&2; exit 1; }

PROFILE=false
ROUTE=roblab-long
OUTPUT_FILES_PREFIX=/tmp/standalone_

while getopts "po:r:" OPT; do
	case "$OPT" in
		p)
			# Run planner under profiler and then visualize the data.
			PROFILE=true
			;;
		o)
			# Where to write the debug files to.
			OUTPUT_FILES_PREFIX="$OPTARG"
			;;
		r)
			# Select the route to plan, from "routes.env".
			ROUTE="$OPTARG"
			;;
		*)
			usage
			;;
	esac
done

# The directory where this script itself resides.
SCRIPT_PATH=$(readlink -f `dirname "$0"`)

# We assume that the working directory is the directory to build,
# and call CMake (if needed) and then make.
if [ ! -f "Makefile" ]; then
	cmake "$SCRIPT_PATH"
fi
make -j4

if [ "$ROUTE" = ALL ]; then
	# Special route name, run this script on all known routes.

	ALL_ROUTES=$(sed -n 's/.*ROUTE" = "\(.*\)".*/\1/p' "$SCRIPT_PATH/routes.env")

	PROFILE_ARG=""
	if [ "$PROFILE" = true ]; then
		PROFILE_ARG="-p"
	fi

	for ROUTE in $ALL_ROUTES; do
		echo ""
		echo "Running on route '$ROUTE'..."
		echo ""

		OUTPUT_ARG="-o /tmp/standalone_route-${ROUTE}_"

		"./$0" $PROFILE_ARG $OUTPUT_ARG -r "$ROUTE"
	done

	exit 0
fi

# Get map and coordinates for the selected route.
source "$SCRIPT_PATH/routes.env"
MAP="$SCRIPT_PATH/../map/$MAP"

INFLATION_RADIUS="1.0"
ANGLE_GRANULARITY=24

if [ "$PROFILE" = true ]; then
	# Remove old profiling data, if there is any.
	rm -f perf.data

	# Command to collect performance data.
	PERF="perf record --call-graph dwarf --"

	# No generation of images, and single threaded only, to make
	# it easier to interpret the data in the profiler.
	WRITE_DEBUG_IMAGES=false
	MULTI_THREADED=false
else
	# Remove old debug files, if there are any.
	rm -f "$OUTPUT_FILES_PREFIX*.png" "$OUTPUT_FILES_PREFIX*.csv"

	PERF="time -p"
	WRITE_DEBUG_IMAGES=true
	MULTI_THREADED=true
fi

# Prevent OpenCV from printing some messages to the terminal.
export OPENCV_LOG_LEVEL=OFF

$PERF ./trajectory_planner_standalone \
    --multi_threaded "$MULTI_THREADED" \
    --angle_granularity "$ANGLE_GRANULARITY" \
    --map "$MAP" \
    --map_resolution "$MAP_RESOLUTION" \
    --inflation_radius "$INFLATION_RADIUS" \
    --start_x "$START_X" \
    --start_y "$START_Y" \
    --start_theta "$START_THETA" \
    --goal_x "$GOAL_X" \
    --goal_y "$GOAL_Y" \
    --goal_theta "$GOAL_THETA" \
    --write_debug_images "$WRITE_DEBUG_IMAGES" \
    --debug_files_prefix "$OUTPUT_FILES_PREFIX"

if [ "$PROFILE" = true ]; then
	# Visualize collected data.
	hotspot perf.data
else
	for CSV in $OUTPUT_FILES_PREFIX*.csv; do
		[ -e "$CSV" ] || continue

		echo "creating plots from '$CSV'..."
		"$SCRIPT_PATH/../scripts/plot_path.py" "$CSV"
	done
fi
