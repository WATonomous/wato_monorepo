from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_traffic_light = LaunchConfiguration("launch_traffic_light", default=True)
    launch_traffic_light_arg = DeclareLaunchArgument(
        "launch_traffic_light",
        default_value=launch_traffic_light,
        description="Launch traffic light detection",
    )
    launch_traffic_signs = LaunchConfiguration("launch_traffic_signs", default=True)
    launch_traffic_signs_arg = DeclareLaunchArgument(
        "launch_traffic_signs",
        default_value=launch_traffic_signs,
        description="Launch traffic signs detection",
    )

    launch_args = [launch_traffic_light_arg, launch_traffic_signs_arg]

    camera_object_detection_launch_include_dir = os.path.join(
        get_package_share_directory("camera_object_detection"), "launch", "include"
    )

    pretrained_yolov8_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [camera_object_detection_launch_include_dir, "/pretrained_yolov8.launch.py"]
        ),
    )

    traffic_light_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [camera_object_detection_launch_include_dir, "/traffic_light.launch.py"]
        ),
        condition=LaunchConfigurationEquals("launch_traffic_light", "True"),
    )

    traffic_signs_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [camera_object_detection_launch_include_dir, "/traffic_signs.launch.py"]
        ),
        condition=LaunchConfigurationEquals("launch_traffic_signs", "True"),
    )

    post_synchronizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [camera_object_detection_launch_include_dir, "/post_synchronizer.launch.py"]
        )
    )

    return LaunchDescription(
        launch_args
        + [
            pretrained_yolov8_launch,
            traffic_light_launch,
            traffic_signs_launch,
            post_synchronizer_launch,
        ]
    )
