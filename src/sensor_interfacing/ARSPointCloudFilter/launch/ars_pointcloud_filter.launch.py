from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals



def generate_launch_description():
    """Launch ARSPointCloudFilter node."""
    return LaunchDescription([
        DeclareLaunchArgument('filter_mode', default_value='ars'),
        Node(
            LaunchConfiguration('filter_mode'),
            condition=LaunchConfigurationEquals('filter_mode', 'ars'),
            package = 'ars_pointcloud_filter',
            executable = 'ars_pointcloud_filter_node'
        )
    ])
