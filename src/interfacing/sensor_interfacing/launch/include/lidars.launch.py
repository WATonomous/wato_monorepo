import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, EmitEvent
from launch.events import Shutdown
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node


def generate_launch_description():
    # Referenced from https://github.com/ros-drivers/velodyne/blob/ros2/velodyne_driver/launch/velodyne_driver_node-VLP32C-launch.py
    velodyne_config_directory = os.path.join(get_package_share_directory(
        'velodyne_driver'), 'config')
    params = os.path.join(
        velodyne_config_directory, 'VLP32C-velodyne_driver_node-params.yaml')
    velodyne_driver_node = Node(package='velodyne_driver',
                                executable='velodyne_driver_node',
                                output='both',
                                parameters=[params])

    return LaunchDescription([
        velodyne_driver_node,
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[EmitEvent(
                    event=Shutdown())],
            )),
    ])
