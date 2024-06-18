from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("camera_object_detection"), 
        "config", 
        "combined_config.yaml"
    )

    left_combined_detection_node = Node(
        package="camera_object_detection",
        executable="camera_object_detection_node",
        name="left_combined_detection_node",
        parameters=[config],
    )

    center_combined_detection_node = Node(
        package="camera_object_detection",
        executable="camera_object_detection_node",
        name="center_combined_detection_node",
        parameters=[config],
    )

    right_combined_detection_node = Node(
        package="camera_object_detection",
        executable="camera_object_detection_node",
        name="right_combined_detection_node",
        parameters=[config],
    )

    return LaunchDescription(
        [
            left_combined_detection_node,
            center_combined_detection_node,
            right_combined_detection_node
        ]
    )
