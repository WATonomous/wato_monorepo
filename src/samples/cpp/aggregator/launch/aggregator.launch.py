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

    aggregator_node = Node(
        package='aggregator',
        executable='aggregator_node',
        parameters=[config],
    )

    ld.add_action(aggregator_node)
    return ld
