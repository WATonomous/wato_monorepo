from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Launch tracking node."""
    tracking_param_file = PathJoinSubstitution([
        FindPackageShare('tracking_2d'),
        'config',
        'params.yaml'
    ])

    tracking_2d_node = Node(
        package='tracking_2d',
        name='tracking_2d_node',
        executable='tracking_2d_node',
        parameters=[tracking_param_file]
    )

    return LaunchDescription([
        tracking_2d_node
    ])
