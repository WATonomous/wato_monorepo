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
        package='pubsub_sample_cpp',
        executable='producer',
        parameters=[config],
    )

    transformer_node = Node(
        package='pubsub_sample_cpp',
        executable='transformer',
        parameters=[config],
    )

    aggregator_node = Node(
        package='pubsub_sample_cpp',
        executable='aggregator',
    )

    ld.add_action(producer_node)
    ld.add_action(transformer_node)
    ld.add_action(aggregator_node)

    return ld
