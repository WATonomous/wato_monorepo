from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([
        Node(
            package='lane_detection',
            executable='lane_detection',
            name='lane_detection',
        )
    ])
