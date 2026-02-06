#!/bin/bash
# Kill any existing ROS2/Gazebo processes
pkill -f "ros2"
pkill -f "gz sim"
pkill -f "ruby"
pkill -f "python3"

echo "Building ROS2 Workspace..."
cd /home/roha/Projects/yolo_pose_ws

# Add virtual environment site-packages to PYTHONPATH
export PYTHONPATH=$PYTHONPATH:/home/roha/Projects/yolo_pose_ws/virtual/lib/python3.12/site-packages

# Build the package
colcon build --packages-select pose_detection_pkg
source install/setup.bash

echo "Launching Simulation..."
# Launch Gazebo + YOLO Node + RViz
ros2 launch pose_detection_pkg pose_detection.launch.py \
    model:="/home/roha/Downloads/horse_26kp-20260206T005238Z-1-001/horse_26kp/runs/pose/runs/pose/yolov26s-pose-horse/weights/best.pt" \
    use_sim:=true
