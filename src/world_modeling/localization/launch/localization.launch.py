from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch localization node."""
    config_file = LaunchConfiguration('config', default_value='./config/config.yaml')   #getting config we defined in config.yaml


    return LaunchDescription([
        # Declare the launch argument
        DeclareLaunchArgument(
            'config',
            default_value = config_file,
        ),
        
        Node(
            package='robot_localization',
            executable='ekf_localization_node',
            name='ekf_localization_node',
            parameters=[config_file],
            output='screen'
        )
    ])

