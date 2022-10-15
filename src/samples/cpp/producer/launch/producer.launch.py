from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        os.getcwd(),
        'src',
        'config',
        'params.yaml',
    )

    producer_node = Node(
        package='producer',
        executable='producer_node',
        parameters=[config],
    )

    ld.add_action(producer_node)
    return ld
