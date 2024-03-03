from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription(
        [
            launch_ros.actions.Node(
                namespace="carla_sample_node",
                package='carla_sample_node',
                executable='carla_sample_node',
                output='screen'),
        ])
