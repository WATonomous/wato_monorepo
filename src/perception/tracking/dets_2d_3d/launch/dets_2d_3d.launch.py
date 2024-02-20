from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch aggregator node."""
    tracking_node = Node(
        package='dets_2d_3d',
        executable='dets_2d_3d_node',
    )

    return LaunchDescription([
        tracking_node
    ])
