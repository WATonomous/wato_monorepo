#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_det_py',
            executable='inference',
            name='inference',
            output='screen',
        ),
    ])
