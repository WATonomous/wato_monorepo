import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch occupancy node."""
    occupancy_pkg_prefix = get_package_share_directory('occupancy')
    occupancy_param_file = os.path.join(
        occupancy_pkg_prefix, 'config', 'params.yaml')

    occupancy_param = DeclareLaunchArgument(
        'occupancy_param_file',
        default_value=occupancy_param_file,
        description='Path to config file for occupancy node'
    )

    occupancy_node = Node(
        package='occupancy',
        executable='occupancy_node',
        parameters=[LaunchConfiguration('occupancy_param_file')],
    )

    return LaunchDescription([
        occupancy_param,
        occupancy_node
    ])
