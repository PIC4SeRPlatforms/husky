'''
Copyright (C) 2015-2023 Swift Navigation Inc.
Contact: https://support.swiftnav.com

This source is subject to the license found in the file 'LICENSE' which must
be be distributed together with this source. All other rights reserved.

THIS CODE AND INFORMATION IS PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A PARTICULAR PURPOSE.
'''

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, ThisLaunchFileDir


def generate_launch_description():

    ld = LaunchDescription()
    
    config = PathJoinSubstitution([ThisLaunchFileDir(), "..", "config", "dual_gnss.yaml"])

    reference = Node(
        namespace='piksi',
        name='swiftnav_ros2_driver_reference',
        package='swiftnav_ros2_driver',
        executable='sbp-to-ros',
        parameters=[config]
    )

    baseline = Node(
        namespace='piksi',
        name='swiftnav_ros2_driver_baseline',
        package='swiftnav_ros2_driver',
        executable='sbp-to-ros',
        parameters=[config]
    )


    ld.add_action(reference)
    ld.add_action(baseline)
    return ld

