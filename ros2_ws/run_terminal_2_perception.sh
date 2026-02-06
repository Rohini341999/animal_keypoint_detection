#!/usr/bin/env bash
set -eo pipefail

# Avoid nounset issues inside ROS setup scripts, then re-enable strict mode.
: "${AMENT_TRACE_SETUP_FILES:=}"
set +u

# Terminal 2: source ROS2 Jazzy base environment.
source /opt/ros/jazzy/setup.bash

# Terminal 2: keep ROS log files inside the workspace for reproducibility.
export ROS_LOG_DIR="$(dirname "$0")/.ros_log"
mkdir -p "$ROS_LOG_DIR"

# Terminal 2: fail early with a clear hint if Python env is not prepared.
if [[ ! -f "$(dirname "$0")/virtual/bin/activate" ]]; then
  echo "Missing virtual environment. Run ./install_python_deps.sh first." >&2
  exit 1
fi

# Terminal 2: activate local Python environment with ultralytics + pinned numpy.
source "$(dirname "$0")/virtual/bin/activate"

# Terminal 2: source this workspace to expose package messages/nodes.
source "$(dirname "$0")/install/setup.bash"

# Terminal 2: run YOLO keypoint inference + custom-msg overlay processing.
ros2 launch horse_pose_sim perception.launch.py \
  use_rviz:=false \
  model_path:="$(dirname "$0")/yolo_models/yolov11n-pose-horse-best-overall.pt"
