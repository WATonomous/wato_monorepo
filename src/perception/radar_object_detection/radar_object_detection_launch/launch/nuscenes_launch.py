from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('radar_object_detection'),
        'config',
        'nuscenes_config.yaml'
    )

    # nodes
    radar_detection = Node(
        package='radar_vis',
        executable='radar_vis_node',
        name='radar_velocity_detection_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # finalize
    ld.add_action(radar_detection)

    return ld