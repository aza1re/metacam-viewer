#!/bin/bash

set -e

OUTPUT_DIR="/home/user/metacam-edu-reader/output"
TEMP_DIR="$1"

if [ -z "$TEMP_DIR" ]; then
    echo "Usage: $0 <data_folder>"
    exit 1
fi

# Remove the whole output folder before starting
if [ -d "$OUTPUT_DIR" ]; then
    echo "==> Removing existing output directory: $OUTPUT_DIR"
    rm -rf "$OUTPUT_DIR"
fi

mkdir -p "$OUTPUT_DIR/pointcloud"

CALIB_SRC="$TEMP_DIR/info/calibration.json"
CALIB_DST="$OUTPUT_DIR/calibration.json"

if [ -f "$CALIB_SRC" ]; then
    echo "==> Copying calibration.json from $CALIB_SRC to $CALIB_DST"
    cp "$CALIB_SRC" "$CALIB_DST"
else
    echo "Warning: calibration.json not found in $CALIB_SRC"
fi

echo "==> Checking if ROS workspace is already built..."
cd fisheye_ws

if [ -f "devel/setup.bash" ] && [ -d "build" ]; then
    echo "ROS workspace already built. Skipping catkin_make."
else
    echo "Building ROS workspace..."
    source /opt/ros/noetic/setup.bash
    catkin_make
fi

source devel/setup.bash

# Find all .bag files in the given directory (recursively)
BAG_FILES=($(find "$TEMP_DIR" -type f -name "*.bag" | sort -V))
if [ ${#BAG_FILES[@]} -eq 0 ]; then
    echo "No .bag files found in $TEMP_DIR"
    exit 1
fi

echo "==> Running the full preprocessing pipeline for all bag files ..."

export BAG_OUTPUT_DIR_ABS="${OUTPUT_DIR}"
roslaunch /home/user/metacam-edu-reader/fisheye_ws/run.launch &
PIPELINE_PID=$!

# Wait for ROS master to be available
echo "==> Waiting for ROS master to start..."
until rostopic list >/dev/null 2>&1; do
    sleep 1
done

for BAG_FILE in "${BAG_FILES[@]}"; do
    echo "==> Playing bag file: $BAG_FILE"
    rosbag play "$BAG_FILE"
done

sleep 5
kill $PIPELINE_PID
wait $PIPELINE_PID

echo "==> Finished processing all bag files"
echo "==> Preprocessing pipeline complete. Output saved to: $OUTPUT_DIR"