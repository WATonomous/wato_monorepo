# controller_pkg/launch/controller.launch.py
import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # point to your params.yaml under controller_pkg/config
    params_file = os.path.join(
        get_package_share_directory('controller_pkg'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='controller_pkg',
            executable='controller_node',
            name='controller_node',
            output='screen',
            parameters=[params_file]
        )
    ])
