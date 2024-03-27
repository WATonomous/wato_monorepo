from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory("motion_forecasting"), "config", "motion_forecasting_config.yaml"
    )
    motion_forecasting = LaunchConfiguration("motion_forecasting", default=True)
    motion_forecasting_arg = DeclareLaunchArgument(
        "motion_forecasting",
        default_value=motion_forecasting,
        description="Launch motion forecasting",
    )

    launch_args = [motion_forecasting_arg]

    node = Node(
        package="motion_forecasting",
        executable="motion_forecasting_node",
        name="motion_forecasting_node",
        parameters=[config],
    )
    
    return LaunchDescription(
        launch_args
        + [
            node
        ]
    )
