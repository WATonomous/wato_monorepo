from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory('tracking'),
        'config',
        'params.yaml'
    )

    resource_path = os.path.join(
        get_package_share_directory('tracking'),
        'resource'
    )

    tracker_node = Node(
        package='tracking',
        executable='tracker_node',
        name='tracker_node',
        parameters=[config,
                    {'resource_path': resource_path}]
    )
    
    visualization_node = Node(
        package='tracking',
        executable='tracker_visualization',
        name='tracker_viz',
        parameters=[config,
                    {'resource_path': resource_path}]
    )
    
    detection_publisher = Node(
        package='tracking',
        executable='detection_publisher',
        name='detection_publisher',
        parameters=[config,
                    {'resource_path': resource_path}]
    )

    # finalize
    return LaunchDescription([tracker_node, visualization_node])
