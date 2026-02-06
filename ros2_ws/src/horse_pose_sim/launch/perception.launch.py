#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("horse_pose_sim")

    default_model_path = "/home/roha/Documents/horse_26kp/ros2_ws/yolo_models/yolov26s-pose-horse-best.pt"
    default_debug_dir = "/home/roha/Documents/horse_26kp/ros2_ws/debug_frames"

    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    confidence = LaunchConfiguration("confidence")
    debug_save_dir = LaunchConfiguration("debug_save_dir")
    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    yolo_node = Node(
        package="horse_pose_sim",
        executable="horse_pose_node.py",
        name="horse_pose_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "model_path": model_path,
                "image_topic": image_topic,
                "confidence_threshold": confidence,
                "detections_topic": "/horse_keypoints/detections",
                "annotated_image_topic": "/horse_keypoints/annotated",
                "publish_annotated_image": True,
                "debug_save_dir": debug_save_dir,
                "debug_save_every_n": 45,
            }
        ],
    )

    overlay_node = Node(
        package="horse_pose_sim",
        executable="keypoint_overlay_node.py",
        name="keypoint_overlay_node",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "image_topic": image_topic,
                "detections_topic": "/horse_keypoints/detections",
                "overlay_topic": "/horse_keypoints/overlay_from_msg",
                "max_age_sec": 0.35,
            }
        ],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_horse_pose",
        output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "horse_pose.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value=default_model_path,
                description="Path to YOLO pose model (.pt)",
            ),
            DeclareLaunchArgument(
                "image_topic",
                default_value="/camera/image_raw",
                description="ROS image topic from TurtleBot3 camera",
            ),
            DeclareLaunchArgument(
                "confidence",
                default_value="0.25",
                description="YOLO confidence threshold",
            ),
            DeclareLaunchArgument(
                "debug_save_dir",
                default_value=default_debug_dir,
                description="Optional folder for periodic debug frames",
            ),
            DeclareLaunchArgument(
                "use_rviz", default_value="true", description="Launch RViz2"
            ),
            DeclareLaunchArgument(
                "use_sim_time", default_value="true", description="Use simulation clock"
            ),
            yolo_node,
            overlay_node,
            rviz_node,
        ]
    )
