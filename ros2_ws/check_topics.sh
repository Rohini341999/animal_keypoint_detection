#!/usr/bin/env bash
set -eo pipefail

# Avoid nounset issues inside ROS setup scripts, then re-enable strict mode.
: "${AMENT_TRACE_SETUP_FILES:=}"
set +u

# Source ROS2 Jazzy and this workspace.
source /opt/ros/jazzy/setup.bash
source "$(dirname "$0")/install/setup.bash"

# Keep ROS log files local to workspace.
export ROS_LOG_DIR="$(dirname "$0")/.ros_log"
mkdir -p "$ROS_LOG_DIR"

# Use ripgrep when available, otherwise fallback to grep.
if command -v rg >/dev/null 2>&1; then
  topic_filter_cmd='rg -E "camera|horse_keypoints|pose_estimation|tf|odom"'
else
  topic_filter_cmd='grep -E "camera|horse_keypoints|pose_estimation|tf|odom"'
fi

# Show key topics and endpoint counts for camera + keypoint pipeline.
printf '\n[Topic list]\n'
ros2 topic list | eval "$topic_filter_cmd" || true

printf '\n[Topic info] /camera/image_raw\n'
ros2 topic info /camera/image_raw || true

printf '\n[Topic info] /horse_keypoints/detections\n'
ros2 topic info /horse_keypoints/detections || true

printf '\n[Topic info] /horse_keypoints/overlay_from_msg\n'
ros2 topic info /horse_keypoints/overlay_from_msg || true

printf '\n[Topic rate] /camera/image_raw (5s sample)\n'
timeout 5s ros2 topic hz /camera/image_raw --wall-time || true

printf '\n[Camera sample] /camera/image_raw --once --no-arr\n'
ros2 topic echo --once --timeout 5 --no-arr /camera/image_raw --qos-profile sensor_data || true

printf '\n[Overlay sample] /horse_keypoints/overlay_from_msg --once --no-arr\n'
ros2 topic echo --once --timeout 5 --no-arr /horse_keypoints/overlay_from_msg --qos-profile sensor_data || true

printf '\n[Sample messages] /horse_keypoints/detections --once --no-arr\n'
ros2 topic echo --once --timeout 5 --no-arr /horse_keypoints/detections || true
