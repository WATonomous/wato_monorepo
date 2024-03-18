import launch
from launch_ros.actions import launch_ros

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
