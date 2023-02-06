from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch transformer node."""
    transformer_pkg_prefix = get_package_share_directory('transformer')
    transformer_param_file = os.path.join(
        transformer_pkg_prefix, 'config', 'params.yaml')

    transformer_param = DeclareLaunchArgument(
        'transformer_param_file',
        default_value=transformer_param_file,
        description='Path to config file for transformer node'
    )

    transformer_node = Node(
        package='transformer',
        executable='transformer_node',
        parameters=[LaunchConfiguration('transformer_param_file')],
    )

    return LaunchDescription([
        transformer_param,
        transformer_node
    ])
