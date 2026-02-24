import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    li_node = Node(
        package= "lidar_imu_init",
        name= "laser_mapping",
        executable="li_init",
        output = "screen",
        parameters=[{
            "point_filter_num": 3,
            "max_iteration": 5,
            "cube_side_length": 2000.0
        },
        os.path.join(get_package_share_directory("lidar_imu_init"),"config","airy.yaml")
        ]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('lidar_imu_init'),
            'rviz_cfg', 'airy.rviz'
        ])]
    )

    ld =LaunchDescription()
    ld.add_action(li_node)
    ld.add_action(rviz_node)

    return ld