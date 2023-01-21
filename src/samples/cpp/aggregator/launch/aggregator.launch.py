from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch aggregator node."""
    aggregator_node = Node(
        package='aggregator',
        executable='aggregator_node',
    )

    return LaunchDescription([
        aggregator_node
    ])
