from launch import LaunchDescription
from launch_ros.actions import Node

import os


def generate_launch_description():
    ld = LaunchDescription()

    aggregator_node = Node(
        package='aggregator',
        executable='aggregator_node',
    )

    ld.add_action(aggregator_node)
    return ld
