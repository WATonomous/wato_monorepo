from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('lidar_object_detection'),
        'config',
        'eve_config.yaml'
    )

    # nodes
    lidar_object_detection = Node(
        package='lidar_object_detection',
        executable='lidar_object_detection_node',
        name='lidar_object_detection_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # finalize
    ld.add_action(lidar_object_detection)

    return ld
