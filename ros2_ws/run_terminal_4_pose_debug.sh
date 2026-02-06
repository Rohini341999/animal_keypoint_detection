#!/usr/bin/env bash
set -eo pipefail

# Avoid nounset issues inside ROS setup scripts, then re-enable strict mode.
: "${AMENT_TRACE_SETUP_FILES:=}"
set +u

# Terminal 4: source ROS2 Jazzy base environment.
source /opt/ros/jazzy/setup.bash

# Terminal 4: keep ROS log files inside the workspace for reproducibility.
export ROS_LOG_DIR="$(dirname "$0")/.ros_log"
mkdir -p "$ROS_LOG_DIR"

# Terminal 4: inspect raw Gazebo pose stream for all world entities.
# Use this to verify horse placement/orientation numerically when visuals look wrong.
gz topic -e -t /world/horse_arena/pose/info
