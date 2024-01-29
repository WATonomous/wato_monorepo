from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch occupancy_segmentation node."""
    occupancy_segmentation_pkg_prefix = get_package_share_directory('occupancy_segmentation')
    occupancy_segmentation_param_file = os.path.join(
        occupancy_segmentation_pkg_prefix, 'config', 'params.yaml')

    occupancy_segmentation_param = DeclareLaunchArgument(
        'occupancy_segmentation_param_file',
        default_value=occupancy_segmentation_param_file,
        description='Path to config file for occupancy_segmentation node'
    )

    occupancy_segmentation_node = Node(
        package='occupancy_segmentation',
        name='occupancy_segmentation_node',
        executable='occupancy_segmentation_node',
        parameters=[LaunchConfiguration('occupancy_segmentation_param_file')],
    )

    return LaunchDescription([
        occupancy_segmentation_param,
        occupancy_segmentation_node
    ])
