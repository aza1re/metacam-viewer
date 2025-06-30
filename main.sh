#!/bin/bash

set -e

OUTPUT_DIR="output"
BAG_FILE="$1"

if [ -z "$BAG_FILE" ]; then
    echo "Usage: $0 <path_to_bag_file>"
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

echo "==> Running the full preprocessing pipeline..."
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

echo "==> Preprocessing pipeline complete. Output saved to: $OUTPUT_DIR"