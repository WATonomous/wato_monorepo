from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch average_filter node."""
    average_filter_pkg_prefix = get_package_share_directory('average_filter')
    average_filter_param_file = os.path.join(
        average_filter_pkg_prefix, 'config', 'params.yaml')

    average_filter_param = DeclareLaunchArgument(
        'average_filter_param_file',
        default_value=average_filter_param_file,
        description='Path to config file for transformer node'
    )

    average_filter_node = Node(
        package='average_filter',
        executable='average_filter_node',
        parameters=[LaunchConfiguration('average_filter_param_file')],
    )

    return LaunchDescription([
        average_filter_param,
        average_filter_node
    ])
