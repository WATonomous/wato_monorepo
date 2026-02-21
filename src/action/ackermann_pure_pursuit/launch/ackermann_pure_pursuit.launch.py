from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("ackermann_pure_pursuit")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")

    return LaunchDescription(
        [
            Node(
                package="ackermann_pure_pursuit",
                executable="pure_pursuit_node",
                name="pure_pursuit_node",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
