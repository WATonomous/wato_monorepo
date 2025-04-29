from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    config_file = PathJoinSubstitution([
        FindPackageShare('bbox_2d_3d'),
        'config',
        'params.yaml'
    ])

    bbox_2d_3d_node = Node(
        package='bbox_2d_3d',
        executable='bbox_2d_3d_node',
        name='bbox_2d_3d_node',
        parameters=[config_file]
    )

    return LaunchDescription([
        bbox_2d_3d_node
    ])
