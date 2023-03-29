from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch projector node."""
    projector_pkg_prefix = get_package_share_directory('projector')
    projector_param_file = os.path.join(
        projector_pkg_prefix, 'config', 'params.yaml')

    projector_param = DeclareLaunchArgument(
        'projector_param_file',
        default_value=projector_param_file,
        description='Path to config file for projector node'
    )

    projector_node = Node(
        package='projector',
        executable='projector_node',
        parameters=[LaunchConfiguration('projector_param_file')],
    )

    return LaunchDescription([
        projector_param,
        projector_node
    ])
