from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('transformer'),
        'config',
        'params.yaml',
    )

    transformer_node = Node(
        package='transformer',
        executable='transformer_node',
        parameters=[config],
    )

    ld.add_action(transformer_node)
    return ld
