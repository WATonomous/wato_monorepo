from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Radar Rviz node."""
    radar_rviz_node = Node(
        package='radar_rviz',
        executable='radar_rviz_node',
    )

    return LaunchDescription([
        radar_rviz_node
    ])
