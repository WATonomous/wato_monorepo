from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('semantic_segmentation'),
        'config',
        'params.yaml'
    )
    
    semantic_segmentation_node = Node(
            package='semantic_segmentation',
            executable='semantic_segmentation_node',
            name='semantic_segmentation_node',
            parameters=[config]
        )

    # finalize
    ld.add_action(semantic_segmentation_node)

    return ld
