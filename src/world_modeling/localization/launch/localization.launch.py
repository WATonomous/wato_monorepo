from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch localization node."""
    config_file_path = os.path.join(
        get_package_share_directory('localization'),
        'config',
        'config.yaml'
    )


    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'config',
            default_value = config_file_path,
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node_global',
            parameters=[config_file_path],
            remappings=[("odometry/filtered", "odometry/filtered")],
            output='screen'
        ),
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            parameters=[config_file_path],
            output='screen',
            remappings=[
                ("imu/data", "carla/ego/imu"),
                ("gps/fix", "carla/ego/gnss"),
                ("gps/filtered", "gps/filtered"),
                ("odometry/gps", "odometry/gps"),
                ("odometry/filtered", "odometry/filtered"),
            ],
        )
    ])

