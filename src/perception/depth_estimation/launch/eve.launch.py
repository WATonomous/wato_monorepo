from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('depth_estimation'),
        'config',
        'config.yaml'
    )

    depth_estimation_node = Node(
        package='depth_estimation',
        executable='depth_estimation_node',
        name='depth_estimation_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    ld.add_action(depth_estimation_node)

    return ld
