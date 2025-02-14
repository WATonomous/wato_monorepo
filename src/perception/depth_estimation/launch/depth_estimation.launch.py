import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch depth_estimation node."""
    depth_estimation_pkg_prefix = get_package_share_directory('depth_estimation')
    depth_estimation_param_file = os.path.join(
        depth_estimation_pkg_prefix, 'config', 'config.yaml')

    depth_estimation_param = DeclareLaunchArgument(
        'depth_estimation_param_file',
        default_value=depth_estimation_param_file,
        description='Path to config file for depth_estimation node'
    )

    depth_estimation_node = Node(
        package='depth_estimation',
        executable='depth_estimation_node',
        parameters=[LaunchConfiguration('depth_estimation_param_file')],
    )

    return LaunchDescription([
        depth_estimation_param,
        depth_estimation_node
    ])