from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('camera_object_detection'),
        'config',
        'traffic_light_config.yaml'
    )

    left_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_object_detection_node',
        name='left_traffic_light_node',
        parameters=[config]
    )

    center_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_object_detection_node',
        name='center_traffic_light_node',
        parameters=[config]
    )

    right_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_object_detection_node',
        name='right_traffic_light_node',
        parameters=[config]
    )

    return LaunchDescription(
        [
            left_camera_object_detection_node,
            center_camera_object_detection_node,
            right_camera_object_detection_node
        ]
    )
