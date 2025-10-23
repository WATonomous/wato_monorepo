from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Launch tracking node."""
    tracking_param_file = PathJoinSubstitution([
        FindPackageShare('track_viz_2d'),
        'config',
        'params.yaml'
    ])

    track_viz_2d_node = Node(
        package='track_viz_2d',
        name='track_viz_2d_node',
        executable='track_viz_2d_node',
        parameters=[tracking_param_file]
    )

    return LaunchDescription([
        track_viz_2d_node
    ])
