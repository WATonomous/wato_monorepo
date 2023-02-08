from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch continental driver node."""
    continental_node = Node(
        package='continental_driver',
        executable='continental_node',
    )

    return LaunchDescription([
        continental_node
    ])
