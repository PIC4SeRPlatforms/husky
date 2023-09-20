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

import os
import yaml

import ament_index_python.packages
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ld = LaunchDescription()
    
    dual_gnss_enabled = LaunchConfiguration("dual_gnss")
    ld.add_action(
        DeclareLaunchArgument("dual_gnss", default_value="true", description="Launch dual gnss")
    )
    
    realsense_enabled = LaunchConfiguration("realsense")
    ld.add_action(
        DeclareLaunchArgument("realsense", default_value="true", description="Launch realsense")
    )
    
    imu_enabled = LaunchConfiguration("imu")
    ld.add_action(
        DeclareLaunchArgument("imu", default_value="true", description="Launch imu")
    )
    
    velodyne_enabled = LaunchConfiguration("velodyne")
    ld.add_action(
        DeclareLaunchArgument("velodyne", default_value="true", description="Launch velodyne")
    )
    
    dual_gnss = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husky_bringup"), "launch", "dual_gnss.launch.py"]
            ),
        ),
        condition=IfCondition(dual_gnss_enabled),
    )

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husky_bringup"), "launch", "realsense.launch.py"]
            ),
        ),
        condition=IfCondition(realsense_enabled),
    )
    
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husky_bringup"), "launch", "imu.launch.py"]
            ),
        ),
        condition=IfCondition(imu_enabled),
    )
    
    velodyne = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("husky_bringup"), "launch", "velodyne.launch.py"]
            ),
        ),
        condition=IfCondition(velodyne_enabled),
    )
    
    ld.add_action(dual_gnss)
    ld.add_action(realsense)
    ld.add_action(imu)
    ld.add_action(velodyne)
       
    return ld
