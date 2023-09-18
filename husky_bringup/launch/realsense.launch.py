import os
import sys
import copy
import yaml

from launch import LaunchDescription, LaunchContext
from launch.actions import OpaqueFunction, DeclareLaunchArgument
from launch.substitutions import (
    LaunchConfiguration,
    ThisLaunchFileDir,
    PathJoinSubstitution,
)
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config = PathJoinSubstitution([ThisLaunchFileDir(), "..", "config", "realsense.yaml"])

    camera_names = ["camera_front", "camera_left", "camera_right"]

    ld = LaunchDescription()

    ld.add_action(
        DeclareLaunchArgument(
            name="log_level",
            default_value="info",
            description="debug log level [DEBUG|INFO|WARN|ERROR|FATAL]",
        )
    )

    for camera_name in camera_names:
        ld.add_action(
            Node(
                package="realsense2_camera",
                namespace=camera_name,
                name="realsense2_camera_node",
                executable="realsense2_camera_node",
                parameters=[config],
                output="screen",
                arguments=[
                    "--ros-args",
                    "--log-level",
                    LaunchConfiguration("log_level"),
                ],
                emulate_tty=True,
            )
        )

    return ld