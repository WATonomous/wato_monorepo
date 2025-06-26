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

    config_file = LaunchConfiguration('localization_param_file')

    odom = Node(
        package='localization',
        executable='odom',
        parameters=[config_file],
    )

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node_global',
        parameters=[config_file],
        remappings=[("odometry/filtered", "odometry/filtered")],
        output='screen'
    ),

    navsat_transform = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[config_file],
        output='screen',
        remappings=[
            ("imu/data", "carla/ego/imu"),
            ("gps/fix", "carla/ego/gnss"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/filtered"),
        ],
    )

    # odom_mock_data = Node(
    #     package='localization',
    #     executable='odom_mock_data',
    #     parameters=[LaunchConfiguration('localization_param_file')],
    # )

    return LaunchDescription([
        localization_param,
        odom,
    ])
