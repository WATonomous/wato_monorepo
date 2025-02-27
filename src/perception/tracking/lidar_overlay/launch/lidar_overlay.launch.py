from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare('lidar_overlay'), 
        'config',  
        'params.yaml' 
    ])

   
    lidar_overlay_node = Node(
        package='lidar_overlay', 
        executable='lidar_overlay_node',  
        name='lidar_overlay_node',  
        parameters=[config_file],  
        arguments=['--ros-args', '--log-level', 'info'], 
    )

 
    return LaunchDescription([
        lidar_overlay_node
    ])