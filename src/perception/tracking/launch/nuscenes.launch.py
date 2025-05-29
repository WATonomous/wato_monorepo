import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch nuscenes publisher node."""
    nusc_pkg_prefix = get_package_share_directory('tracking')
    nusc_param_file = os.path.join(nusc_pkg_prefix, 'config', 'nuscenes_params.yaml')
    
    nusc_param = DeclareLaunchArgument(
        'nusc_param_file',
        default_value=nusc_param_file,
        description='Path to config file for tracking node'
    )

    nusc_node = Node(
        package='tracking',
        name='nusc_publisher',
        executable='nusc_publisher',
        parameters=[LaunchConfiguration('nusc_param_file')]
    )

    return LaunchDescription([
        nusc_param,
        nusc_node
    ])
