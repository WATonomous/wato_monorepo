from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare('spatial_association'),
        'config',
        'params.yaml'
    ])

    spatial_association_node = Node(
        package='spatial_association',
        executable='spatial_association_node',
        name='spatial_association_node',
        parameters=[config_file]
    )

    return LaunchDescription([
        spatial_association_node
    ])
