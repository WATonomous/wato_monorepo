from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_object_detection'),
        'config',
        'post_synchronizer_config.yaml'
    )

    left_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_sync_node',
        name='left_post_synchronizer_node',
        parameters=[config]
    )

    center_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_sync_node',
        name='center_post_synchronizer_node',
        parameters=[config]
    )

    right_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_sync_node',
        name='right_post_synchronizer_node',
        parameters=[config]
    )

    return LaunchDescription(
        [
            left_camera_object_detection_node,
            center_camera_object_detection_node,
            right_camera_object_detection_node
        ]
    )
