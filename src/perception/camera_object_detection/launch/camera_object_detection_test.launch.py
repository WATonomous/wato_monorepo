from ament_index_python import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():

    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('camera_object_detection'),
        'launch',
        'camera_object_detection_test.launch.py'
    )

    # NODES
    camera_object_detection_node = Node(
        package='camera_object_detection',
        executable='camera_object_detection_node',
        name='camera_object_detection_node',
        parameters=[{
            'camera_topic': '/camera_pkg/display_mjpeg',
            'publish_vis_topic': '/annotated_img',
            'publish_obstacle_topic': '/obstacles',
            'model_path': '/perception_models/yolov8s.pt',
            'image_size': 480
        }]
    )

    ld.add_action(camera_object_detection_node)

    return ld
