import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """Launch sample node."""
    hd_map_pkg_prefix = get_package_share_directory('hd_map')
    hd_map_param_file = os.path.join(
        hd_map_pkg_prefix, 'config', 'params.yaml')

    hd_map_param = DeclareLaunchArgument(
        'hd_map_param_file',
        default_value=hd_map_param_file,
        description='Path to config file for hd_map node'
    )

    hd_map_service_node = Node(
        package='hd_map',
        executable='hd_map_service',
        parameters=[LaunchConfiguration('hd_map_param_file')],
    )

    return LaunchDescription([
        hd_map_param,
        hd_map_service_node
    ])
