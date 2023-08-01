from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    turtlesim_node = Node(
        package="lidar_publisher",
        executable="lidar_publisher",
        parameters=[
            {"background_b": 200},
            {"background_g": 200},
            {"background_r": 200}
        ]
    )
    ld.add_action(turtlesim_node)
    return ld