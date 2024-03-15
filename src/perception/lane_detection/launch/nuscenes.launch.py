from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('lane_detection'),
        'config',
        'nuscenes_config.yaml'
    )

    lane_detection_node = Node(
        package='lane_detection',
        executable='lane_detection',
        name='lane_detection_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(lane_detection_node)

    return ld
