import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch tracking node."""
    tracking_pkg_prefix = get_package_share_directory('tracking')
    tracking_param_file = os.path.join(tracking_pkg_prefix, 'config', 'tracking_params.yaml')
    
    tracking_param = DeclareLaunchArgument(
        'tracking_param_file',
        default_value=tracking_param_file,
        description='Path to config file for tracking node'
    )

    tracking_node = Node(
        package='tracking',
        name='tracker_node',
        executable='tracker_node',
        parameters=[LaunchConfiguration('tracking_param_file')]
    )

    nusc_node = Node(
        package='tracking',
        name='nusc_publisher',
        executable='nusc_publisher',
        parameters=[LaunchConfiguration('tracking_param_file')]
    )

    return LaunchDescription([
        tracking_param,
        tracking_node,
        nusc_node
    ])
