from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch simulation node."""
    simulation_node = Node(
        package='simulation',
        executable='simulation_node',
    )

    return LaunchDescription([
        simulation_node
    ])
