#!/usr/bin/env bash
set -eo pipefail

# Avoid nounset issues inside ROS setup scripts, then re-enable strict mode.
: "${AMENT_TRACE_SETUP_FILES:=}"
set +u

# Single terminal option: source ROS2 Jazzy base environment.
source /opt/ros/jazzy/setup.bash

# Single terminal option: keep ROS log files inside the workspace for reproducibility.
export ROS_LOG_DIR="$(dirname "$0")/.ros_log"
mkdir -p "$ROS_LOG_DIR"

# Single terminal option: fail early with a clear hint if Python env is not prepared.
if [[ ! -f "$(dirname "$0")/virtual/bin/activate" ]]; then
  echo "Missing virtual environment. Run ./install_python_deps.sh first." >&2
  exit 1
fi

# Single terminal option: activate Python deps for ultralytics.
source "$(dirname "$0")/virtual/bin/activate"

# Single terminal option: source this workspace.
source "$(dirname "$0")/install/setup.bash"

# Single terminal option: launch simulation + perception + RViz together.
ros2 launch horse_pose_sim full_stack.launch.py \
  model_path:="$(dirname "$0")/yolo_models/yolov26s-pose-horse-best.pt"
