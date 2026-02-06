#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("horse_pose_sim")
    ros_gz_sim_share = get_package_share_directory("ros_gz_sim")
    tb3_share = get_package_share_directory("turtlebot3_gazebo")

    world_default = os.path.join(pkg_share, "worlds", "horse_arena.world")

    use_sim_time = LaunchConfiguration("use_sim_time")
    world = LaunchConfiguration("world")
    use_gui = LaunchConfiguration("use_gui")
    x_pose = LaunchConfiguration("x_pose")
    y_pose = LaunchConfiguration("y_pose")
    tb3_model = LaunchConfiguration("tb3_model")

    gz_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": ["-r -s -v3 ", world],
            "on_exit_shutdown": "true",
        }.items(),
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(ros_gz_sim_share, "launch", "gz_sim.launch.py")),
        launch_arguments={
            "gz_args": "-g -v3",
            "on_exit_shutdown": "true",
        }.items(),
        condition=IfCondition(use_gui),
    )

    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_share, "launch", "robot_state_publisher.launch.py")),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    spawn_tb3 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(tb3_share, "launch", "spawn_turtlebot3.launch.py")),
        launch_arguments={"x_pose": x_pose, "y_pose": y_pose}.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("world", default_value=world_default, description="Path to Gazebo world file"),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation clock"),
            DeclareLaunchArgument("use_gui", default_value="true", description="Launch Gazebo GUI"),
            DeclareLaunchArgument("x_pose", default_value="0.0", description="TurtleBot3 spawn x"),
            DeclareLaunchArgument("y_pose", default_value="0.0", description="TurtleBot3 spawn y"),
            DeclareLaunchArgument("tb3_model", default_value="burger_cam", description="TurtleBot3 model"),
            SetEnvironmentVariable("TURTLEBOT3_MODEL", tb3_model),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(tb3_share, "models")),
            AppendEnvironmentVariable("GZ_SIM_RESOURCE_PATH", os.path.join(pkg_share, "models")),
            gz_server,
            gz_gui,
            robot_state_publisher,
            spawn_tb3,
        ]
    )
