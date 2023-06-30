from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch sample node."""
    pkg_prefix = get_package_share_directory('hd_map')
    sample_param_file = os.path.join(
        pkg_prefix, 'config', 'sample_params.yaml')

    sample_param = DeclareLaunchArgument(
        'sample_param_file',
        default_value=sample_param_file,
        description='Path to config file for sample node'
    )

    hd_map_service_node = Node(
        package='hd_map',
        executable='hd_map_service'
    )

    return LaunchDescription([
        sample_param,
        hd_map_service_node
    ])
