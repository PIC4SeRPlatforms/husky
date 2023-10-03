# Copyright 2018 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    robot_localization_dir = get_package_share_directory("husky_control")
    parameters_file_dir = os.path.join(robot_localization_dir, "config")
    params = os.path.join(parameters_file_dir, "localization.yaml")

    return LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                "output_final_position", default_value="false"
            ),
            launch.actions.DeclareLaunchArgument(
                "output_location", default_value="~/dual_ekf_navsat_example_debug.txt"
            ),
            # launch_ros.actions.Node(
            #         package='husky_localization',
            #         executable='set_datum_node',
            #         name='set_datum_call',
            #         output='screen',
            #         parameters=[{'sim_flag': False}, params],
            #        ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="ekf_node",
                name="ekf_node_map",
                output="screen",
                parameters=[params],
                remappings=[
                    ("odometry/filtered", "odometry/filtered/global"),
                    ("/set_pose", "/set_pose/global"),
                ],
            ),
            launch_ros.actions.Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform",
                output="screen",
                parameters=[params],
                remappings=[
                    ("imu/data", "/microstrain/nav/filtered_imu/data"),
                    ("gps/fix", "/piksi/navsatfix"),
                    ("odometry/filtered", "odometry/filtered/global"),
                ],
            ),
        ]
    )
