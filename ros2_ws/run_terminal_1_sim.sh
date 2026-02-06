#!/usr/bin/env bash
set -eo pipefail

# Avoid nounset issues inside ROS setup scripts, then re-enable strict mode.
: "${AMENT_TRACE_SETUP_FILES:=}"
set +u

# Terminal 1: source ROS2 Jazzy base environment.
source /opt/ros/jazzy/setup.bash

# Terminal 1: source this workspace after it has been built.
source "$(dirname "$0")/install/setup.bash"

# Terminal 1: keep ROS log files inside the workspace for reproducibility.
export ROS_LOG_DIR="$(dirname "$0")/.ros_log"
mkdir -p "$ROS_LOG_DIR"

# Terminal 1: choose TurtleBot3 model with a camera topic (/camera/image_raw).
export TURTLEBOT3_MODEL=burger_cam

# Terminal 1: launch Gazebo world with horse models and spawn TurtleBot3.
ros2 launch horse_pose_sim sim_world.launch.py
