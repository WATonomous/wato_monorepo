from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('camera_object_detection'),
        'config',
        'traffic_signs_config.yaml'
    )

    # nodes
    camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_object_detection_node',
        name='traffic_signs_node',
        parameters=[config]
    )

    # finalize
    ld.add_action(camera_object_detection_node)

    return ld
