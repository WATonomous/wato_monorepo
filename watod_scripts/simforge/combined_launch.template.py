"""After map and truth gates: include generated launch copies in one process."""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource


def generate_launch_description():
    root = os.path.dirname(os.path.abspath(__file__))
    return LaunchDescription([
        IncludeLaunchDescription(AnyLaunchDescriptionSource(os.path.join(root, name)))
        for name in ("world_modeling_simforge.launch.yaml", "action_simforge.launch.yaml")
    ])
