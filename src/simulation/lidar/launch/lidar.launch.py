from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch lidar node."""
    lidar_node = Node(
        package='lidar',
        executable='lidar_node',
    )

    return LaunchDescription([
        lidar_node
    ])
