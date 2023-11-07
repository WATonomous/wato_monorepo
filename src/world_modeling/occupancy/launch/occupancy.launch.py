from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

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

    voxelizer_node = Node(
        package='occupancy',
        executable='voxelizer_node'
    )
    voxel_seg_node = Node(
        package='occupancy',
        executable='voxel_seg_node'
    )

    voxel_segmentation_node = Node(
        package='occupancy',
        executable='voxel_segmentation_node.py'
    )

    return LaunchDescription([
        sample_param,
        sample_node,
        voxel_segmentation_node,
        voxelizer_node,
        voxel_seg_node
    ])
