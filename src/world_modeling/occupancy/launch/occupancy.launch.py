from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    """Launch sample node."""
    pkg_prefix = get_package_share_directory('occupancy')
    sample_param_file = os.path.join(
        pkg_prefix, 'config', 'sample_params.yaml')

    sample_param = DeclareLaunchArgument(
        'sample_param_file',
        default_value=sample_param_file,
        description='Path to config file for sample node'
    )

    sample_node = Node(
        package='occupancy',
        executable='sample_node',
        parameters=[LaunchConfiguration('sample_param_file')],
    )

    voxel_segmentation_node = Node(
        package='occupancy',
        executable='voxel_segmentation_node.py'
    )

    return LaunchDescription([
        sample_param,
        sample_node,
        voxel_segmentation_node
    ])
