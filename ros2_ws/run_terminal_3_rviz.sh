#!/usr/bin/env bash
set -eo pipefail

# Avoid nounset issues inside ROS setup scripts, then re-enable strict mode.
: "${AMENT_TRACE_SETUP_FILES:=}"
set +u

# Terminal 3: source ROS2 Jazzy base environment.
source /opt/ros/jazzy/setup.bash

# Terminal 3: keep ROS log files inside the workspace for reproducibility.
export ROS_LOG_DIR="$(dirname "$0")/.ros_log"
mkdir -p "$ROS_LOG_DIR"

# Terminal 3: source this workspace to load package assets.
source "$(dirname "$0")/install/setup.bash"

# Terminal 3: open RViz2 with predefined image + TF layout for keypoint visualization.
rviz2 -d "$(dirname "$0")/src/horse_pose_sim/rviz/horse_pose.rviz"
