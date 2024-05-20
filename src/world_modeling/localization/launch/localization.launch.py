from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch localization node."""
    aggregator_node = Node(
        package='localization',
        executable='sample_node',
    )

    return LaunchDescription([
        sample_node
    ])
