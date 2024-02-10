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
        get_package_share_directory('camera_object_detection'),
        'config',
        'eve_config.yaml'
    )

    # nodes
    left_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_object_detection_node',
        name='left_camera_object_detection_node',
        parameters=[config]
    )
    
    center_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_object_detection_node',
        name='center_camera_object_detection_node',
        parameters=[config]
    )

    right_camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_object_detection_node',
        name='right_camera_object_detection_node',
        parameters=[config]
    )

    # finalize
    ld.add_action(left_camera_object_detection_node)
    ld.add_action(center_camera_object_detection_node)
    ld.add_action(right_camera_object_detection_node)

    return ld
