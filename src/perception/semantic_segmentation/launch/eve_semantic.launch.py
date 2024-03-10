from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('semantic_segmentation'),
        'config',
        'eve_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='semantic_segmentation',
            executable='semantic_segmentation',
            name='semantic_segmentation_node',
            parameters=[config]
        )
    ])
