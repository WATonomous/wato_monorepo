from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('dets_2d_3d'),
        'config',
        'nuscenes_config.yaml'
    )

    # nodes
    detections_2d_3d_node = Node(
        package='dets_2d_3d',
        executable='dets_2d_3d_node',
        name='dets_2d_3d_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    # finalize
    ld.add_action(detections_2d_3d_node)

    return ld
