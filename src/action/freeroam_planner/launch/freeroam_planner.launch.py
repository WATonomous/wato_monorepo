from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory("freeroam_planner")
    params_file = os.path.join(pkg_dir, "config", "params.yaml")

    return LaunchDescription(
        [
            Node(
                package="freeroam_planner",
                executable="freeroam_planner_node",
                name="freeroam_planner_node",
                output="screen",
                parameters=[params_file],
            )
        ]
    )
