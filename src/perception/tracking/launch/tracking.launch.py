from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Radar object detection system."""
    trackingnode = Node(
        package='tracking',
        executable='tracking_node',
    )

    return LaunchDescription([
        trackingnode
    ])
