from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    camera_compression_node = Node(
        package='camera_compression',
        executable='camera_compression_node',
    )

    return LaunchDescription([
        camera_compression_node
    ])
