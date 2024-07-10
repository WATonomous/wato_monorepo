from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch localization node."""
    localization_node = Node(
        package='localization',
        executable='localization_node',
    )

    return LaunchDescription([
        localization_node
    ])
