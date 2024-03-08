from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("lane_detection"), "config", "nuscenes_config.yaml"
    )

    return LaunchDescription(
        [
            Node(
                package="lane_detection",
                executable="lane_detection",
                name="lane_detection_node",
                parameters=[config],
            )
        ]
    )
