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

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    filepath_config_twist_mux = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "twist_mux.yaml"]
    )

    filepath_config_interactive_markers = PathJoinSubstitution(
        [FindPackageShare("husky_control"), "config", "teleop_interactive_markers.yaml"]
    )

    node_interactive_marker_twist_server = Node(
        package="interactive_marker_twist_server",
        executable="marker_server",
        name="twist_server_node",
        remappings={("cmd_vel", "twist_marker_server/cmd_vel")},
        parameters=[filepath_config_interactive_markers],
        output="screen",
    )

    node_twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        output="screen",
        remappings={("/cmd_vel_out", "/husky_velocity_controller/cmd_vel_unstamped")},
        parameters=[filepath_config_twist_mux],
    )

    ld = LaunchDescription()
    ld.add_action(node_interactive_marker_twist_server)
    ld.add_action(node_twist_mux)
    return ld
