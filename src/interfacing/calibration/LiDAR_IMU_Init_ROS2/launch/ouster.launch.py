import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld =LaunchDescription()
    node = Node(
        package= "lidar_imu_init",
        name= "laser_mapping",
        executable="li_init",
        output = "screen",
        parameters=[{
            "point_filter_num":3,
            "max_iteration":5,
            "cube_side_length": 2000.0, 
        },
        os.path.join(get_package_share_directory("lidar_imu_init"),"config","ouster.yaml")
        ]
    )

    rviz = Node(
       package= "rviz2",
       name = "rviz2",
       executable= "rviz2",
       output ="screen"
   )
    ld.add_action(node)
    ld.add_action(rviz)

    return ld