from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('tracking_viz'),
        'config',
        'basic_config.yaml'
    )

    tracking_viz_node = Node(
        package='tracking_viz',
        executable='tracking_viz_node',
        name='tracking_viz_node',
        parameters=[config],
        arguments=['--ros-args', '--log-level', 'info']
    )

    return LaunchDescription([tracking_viz_node])
