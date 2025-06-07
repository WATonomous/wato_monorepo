from launch import LaunchDescription
import launch_ros.actions

# For creating launch arguments
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    publish_autopilot_arg = DeclareLaunchArgument(
        'publish_autopilot',
        default_value='False'
    )

    return LaunchDescription(
        [
            publish_autopilot_arg,
            launch_ros.actions.Node(
                namespace="carla_sample_node",
                package='carla_sample_node',
                executable='carla_sample_node',
                parameters=[{
                    'publish_autopilot': LaunchConfiguration('publish_autopilot')
                }],
                output='screen')
        ])
