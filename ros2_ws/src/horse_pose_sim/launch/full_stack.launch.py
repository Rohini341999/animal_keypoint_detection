#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    use_rviz = LaunchConfiguration("use_rviz")
    model_path = LaunchConfiguration("model_path")
    confidence = LaunchConfiguration("confidence")

    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("horse_pose_sim"), "launch", "sim_world.launch.py"])
        ),
        launch_arguments={
            "tb3_model": "burger_cam",
            "use_gui": "true",
        }.items(),
    )

    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("horse_pose_sim"), "launch", "perception.launch.py"])
        ),
        launch_arguments={
            "model_path": model_path,
            "confidence": confidence,
            "use_rviz": use_rviz,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "model_path",
                default_value="/home/roha/Documents/horse_26kp/ros2_ws/yolo_models/yolov26s-pose-horse-best.pt",
                description="Path to YOLO pose model",
            ),
            DeclareLaunchArgument("confidence", default_value="0.25", description="YOLO confidence threshold"),
            DeclareLaunchArgument("use_rviz", default_value="true", description="Launch RViz2"),
            sim_launch,
            perception_launch,
        ]
    )
