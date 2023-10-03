# Copyright 2023 Andrea Ostuni - PIC4SeR
#
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

from launch import LaunchContext, LaunchDescription
from launch.substitutions import EnvironmentVariable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    lc = LaunchContext()
    ld = LaunchDescription()

    config_husky_ekf = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "localization.yaml"],
    )

    node_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node_odom",
        output="screen",
        parameters=[config_husky_ekf],
        remappings=[
            ("/set_pose", "/set_pose/local"),
            ("odometry/filtered", "odometry/filtered/local")
        ]
    )
    ld.add_action(node_ekf)

    primary_imu_enable = EnvironmentVariable("CPR_IMU", default_value="false")

    if (primary_imu_enable.perform(lc)) == "true":
        config_imu_filter = PathJoinSubstitution(
            [FindPackageShare("husky_control"), "config", "imu_filter.yaml"],
        )
        node_imu_filter = Node(
            package="imu_filter_madgwick",
            executable="imu_filter_madgwick_node",
            name="imu_filter",
            output="screen",
            parameters=[config_imu_filter],
        )
        ld.add_action(node_imu_filter)

    return ld
