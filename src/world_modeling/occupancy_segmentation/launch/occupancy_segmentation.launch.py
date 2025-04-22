from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch occupancy_seg node."""
    occupancy_seg_pkg_prefix = get_package_share_directory('occupancy_segmentation')
    occupancy_seg_param_file = os.path.join(
        occupancy_seg_pkg_prefix, 'config', 'params.yaml')

    occupancy_seg_param = DeclareLaunchArgument(
        'occupancy_segmentation_param_file',
        default_value=occupancy_seg_param_file,
        description='Path to config file for occupancy segmentation node'
    )

    occupancy_seg_node = Node(
        package='occupancy_segmentation',
        executable='occupancy_segmentation_node',
        parameters=[LaunchConfiguration('occupancy_segmentation_param_file')],
    )

    return LaunchDescription([
        occupancy_seg_param,
        occupancy_seg_node
    ])
