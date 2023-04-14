from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch Radar Centerfusion node."""
    radar_centerfusion_node = Node(
        package='radar_centerfusion',
        executable='radar_centerfusion_node',
    )

    return LaunchDescription([
        radar_centerfusion_node
    ])