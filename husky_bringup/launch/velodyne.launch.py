import os
import yaml

import ament_index_python.packages
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import EnvironmentVariable

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share_bringup = get_package_share_directory('husky_bringup')
    
    configs = os.path.join(pkg_share_bringup, 'config', 'velodyne.yaml')
    convert_share_dir = ament_index_python.packages.get_package_share_directory('velodyne_pointcloud')
    with open(configs, 'r') as f:
        configs = yaml.safe_load(f)
        driver_params = configs['velodyne_driver_node']['ros__parameters']
        convert_params = configs['velodyne_transform_node']['ros__parameters']
        laserscan_params = configs['velodyne_laserscan_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(convert_share_dir, 'params', 'VLP16db.yaml')
    
    velodyne_frame_env = EnvironmentVariable('HUSKY_VELODYNE_FRAME', default_value='velodyne_link')
    driver_params['frame_id'] = velodyne_frame_env
    convert_params['fixed_frame'] = velodyne_frame_env
    convert_params['target_frame'] = velodyne_frame_env

    container = ComposableNodeContainer(
            name='velodyne_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='velodyne_driver',
                    plugin='velodyne_driver::VelodyneDriver',
                    name='velodyne_driver_node',
                    namespace='velodyne',
                    parameters=[driver_params]),
                ComposableNode(
                    package='velodyne_pointcloud',
                    plugin='velodyne_pointcloud::Transform',
                    name='velodyne_transform_node',
                    namespace='velodyne',
                    parameters=[convert_params]),
                ComposableNode(
                    package='velodyne_laserscan',
                    plugin='velodyne_laserscan::VelodyneLaserScan',
                    name='velodyne_laserscan_node',
                    namespace='velodyne',
                    parameters=[laserscan_params]),
            ],
            output='both',
    )

    return LaunchDescription([container])