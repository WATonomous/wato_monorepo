from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
import yaml

def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('camera_detection'),
        'config',
        'nuscenes_config.yaml'
    )

    # nodes
    camera_detection_node = Node(
        package='camera_detection',
        executable='camera_detection_node',
        name='camera_detection_node',
        parameters=[config]
    )

    # finalize
    ld.add_action(camera_detection_node)

    return ld