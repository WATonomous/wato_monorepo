import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch localization node."""
    localization_pkg_prefix = get_package_share_directory('localization')
    localization_param_file = os.path.join(
        localization_pkg_prefix, 'config', 'params.yaml')

    localization_param = DeclareLaunchArgument(
        'localization_param_file',
        default_value=localization_param_file,
        description='Path to config file for localization node'
    )

    odom_node = Node(
        package='localization',
        executable='odom_node',
        parameters=[LaunchConfiguration('localization_param_file')],
    )

    odom_mock_data_node = Node(
        package='localization',
        executable='odom_mock_data_node',
        parameters=[LaunchConfiguration('localization_param_file')],
    )

    return LaunchDescription([
        localization_param,
        odom_node,
        odom_mock_data_node
    ])
