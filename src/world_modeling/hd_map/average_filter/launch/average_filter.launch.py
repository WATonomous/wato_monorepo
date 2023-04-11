from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch sample node."""
    sample_pkg_prefix = get_package_share_directory('average_filter')
    sample_param_file = os.path.join(
        sample_pkg_prefix, 'config', 'params.yaml')

    sample_param = DeclareLaunchArgument(
        'average_filter_param_file',
        default_value=sample_param_file,
        description='Path to config file for sample node'
    )

    sample_node = Node(
        package='average_filter',
        executable='average_filter_node',
        parameters=[LaunchConfiguration('average_filter_param_file')],
    )

    return LaunchDescription([
        sample_param,
        sample_node
    ])
