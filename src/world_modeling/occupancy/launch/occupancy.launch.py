from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('occupancy'),
        'config',
        'occupancy.yaml'
    )

    voxelizer_node = Node(
        package='occupancy',
        executable='voxelizer_node',
        name='voxelizer_node',
        parameters=[config]
    )

    return LaunchDescription([voxelizer_node])
