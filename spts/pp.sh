#!/bin/bash

set -e

OUTPUT_DIR="output"
DATA_DIR="$1"

if [ -z "$DATA_DIR" ]; then
    echo "Usage: $0 <data_folder>"
    exit 1
fi

echo "==> Checking if ROS workspace is already built..."
cd fisheye_ws

# Remove the whole output folder before starting
if [ -d "$OUTPUT_DIR" ]; then
    echo "==> Removing existing output directory: $OUTPUT_DIR"
    rm -rf "$OUTPUT_DIR"
fi

if [ -f "devel/setup.bash" ] && [ -d "build" ]; then
    echo "ROS workspace already built. Skipping catkin_make."
else
    echo "Building ROS workspace..."
    source /opt/ros/noetic/setup.bash
    catkin_make
fi

if [ ! -d "$OUTPUT_DIR" ]; then
    echo "==> Creating output directory: $OUTPUT_DIR"
    mkdir -p "$OUTPUT_DIR"
fi

mkdir -p output/pointcloud
source devel/setup.bash

# Find all .bag files in the given directory (recursively)
BAG_FILES=$(find "$DATA_DIR" -type f -name "*.bag")
if [ -z "$BAG_FILES" ]; then
    echo "No .bag files found in $DATA_DIR"
    exit 1
fi

for BAG_FILE in $BAG_FILES; do
    echo "==> Running the full preprocessing pipeline for $BAG_FILE ..."
    roslaunch run.launch &
    PIPELINE_PID=$!

    # Wait for ROS master to be available
    echo "==> Waiting for ROS master to start..."
    until rostopic list >/dev/null 2>&1; do
        sleep 1
    done

    echo "==> Playing bag file: $BAG_FILE"
    rosbag play "$BAG_FILE"

    sleep 5
    kill $PIPELINE_PID
    wait $PIPELINE_PID

    echo "==> Finished processing $BAG_FILE"
done

echo "==> Preprocessing pipeline complete. Output saved to: $OUTPUT_DIR"