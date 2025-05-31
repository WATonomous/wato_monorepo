import os
import yaml

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # Referenced from https://github.com/ros-drivers/velodyne/blob/ros2/velodyne_driver/launch/velodyne_driver_node-VLP32C-launch.py
    velodyne_config_directory = os.path.join(get_package_share_directory(
        'sensor_interfacing'), 'config')
    params = os.path.join(
        velodyne_config_directory, 'VLP32C.yaml')
    velodyne_driver_node = Node(package='velodyne_driver',
                                executable='velodyne_driver_node',
                                output='both',
                                parameters=[params])

    convert_share_dir = get_package_share_directory('velodyne_pointcloud')
    convert_params_file = os.path.join(
        convert_share_dir, 'config', 'VLP32C-velodyne_transform_node-params.yaml')
    with open(convert_params_file, 'r') as f:
        convert_params = yaml.safe_load(f)['velodyne_transform_node']['ros__parameters']
    convert_params['calibration'] = os.path.join(
        convert_share_dir, 'params', 'VeloView-VLP-32C.yaml')
    velodyne_transform_node = Node(package='velodyne_pointcloud',
                                   executable='velodyne_transform_node',
                                   output='both',
                                   parameters=[convert_params])

    laserscan_share_dir = get_package_share_directory('velodyne_laserscan')
    laserscan_params_file = os.path.join(
        laserscan_share_dir, 'config', 'default-velodyne_laserscan_node-params.yaml')
    velodyne_laserscan_node = Node(package='velodyne_laserscan',
                                   executable='velodyne_laserscan_node',
                                   output='both',
                                   parameters=[laserscan_params_file])

    return LaunchDescription([
        velodyne_driver_node,
        velodyne_transform_node,
        velodyne_laserscan_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[EmitEvent(
                    event=Shutdown())],
            )),
    ])