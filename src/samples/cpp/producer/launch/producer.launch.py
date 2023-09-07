from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch producer node."""
    producer_pkg_prefix = get_package_share_directory('producer')
    producer_param_file = os.path.join(
        producer_pkg_prefix, 'config', 'params.yaml')

    producer_param = DeclareLaunchArgument(
        'producer_param_file',
        default_value=producer_param_file,
        description='Path to config file for producer node'
    )

    producer_node = Node(
        package='producer',
        name='producer_node',
        executable='producer_node',
        parameters=[LaunchConfiguration('producer_param_file')],
    )

    return LaunchDescription([
        producer_param,
        producer_node
    ])
