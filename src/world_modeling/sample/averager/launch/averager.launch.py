from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch averager node."""
    averager_node = Node(
        package='averager',
        executable='averager_node',
    )

    return LaunchDescription([
        averager_node
    ])
