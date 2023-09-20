from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import ThisLaunchFileDir, PathJoinSubstitution

from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare('husky_bringup'), "config", "microstrain_gx5.yaml"]
    )

    return LaunchDescription(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [
                        PathJoinSubstitution(
                            [
                                FindPackageShare("microstrain_inertial_driver"),
                                "launch",
                                "microstrain_launch.py",
                            ]
                        )
                    ]
                ),
                launch_arguments={
                    "params_file": config_file,
                    "namespace": "microstrain",
                    "configure": "true",
                    "activate": "true",
                }.items(),
            )
        ]
    )
