from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Radar object detection system."""
    radar_vis_node = Node(
        package='radar_vis',
        executable='radar_vis_node',
    )

    return LaunchDescription([
        radar_vis_node
    ])
