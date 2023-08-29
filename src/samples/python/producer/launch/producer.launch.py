import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory('producer'), 
        'config', 
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package = 'producer',  
            name = 'producer_node',
            executable = 'producer',
            parameters = [param_file_path]
        )
    ])