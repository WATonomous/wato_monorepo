from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            namespace="aggregator",
            package='aggregator',
            executable='aggregator',
            output='screen',
            emulate_tty=True),
    ])
