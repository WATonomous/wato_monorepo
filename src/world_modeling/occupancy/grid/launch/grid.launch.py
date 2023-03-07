from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch grid node."""
    grid_pkg_prefix = get_package_share_directory('grid')
    grid_param_file = os.path.join(
        grid_pkg_prefix, 'config', 'params.yaml')

    grid_param = DeclareLaunchArgument(
        'grid_param_file',
        default_value=grid_param_file,
        description='Path to config file for grid node'
    )

    grid_node = Node(
        package='grid',
        executable='grid_node',
        parameters=[LaunchConfiguration('grid_param_file')],
    )

    return LaunchDescription([
        grid_param,
        grid_node
    ])
