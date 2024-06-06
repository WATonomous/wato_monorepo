from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # launch_traffic_light = LaunchConfiguration("launch_traffic_light", default=True)
    # launch_traffic_light_arg = DeclareLaunchArgument(
    #     "launch_traffic_light",
    #     default_value=launch_traffic_light,
    #     description="Launch traffic light detection",
    # )
    # launch_traffic_signs = LaunchConfiguration("launch_traffic_signs", default=True)
    # launch_traffic_signs_arg = DeclareLaunchArgument(
    #     "launch_traffic_signs",
    #     default_value=launch_traffic_signs,
    #     description="Launch traffic signs detection",
    # )

    # launch_args = [launch_traffic_light_arg, launch_traffic_signs_arg]

    # camera_object_detection_launch_include_dir = os.path.join(
    #     get_package_share_directory("camera_object_detection"), "launch", "include"
    # )

    # combined_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         [camera_object_detection_launch_include_dir, "/combined_model.launch.py"]
    #     ),
    # )
asdasdad
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

    return LaunchDescription(
        [left_combined_detection_node]
    )
