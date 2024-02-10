import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value=use_sim_time,
                                             description='Running Simulation?')

    urdf_filepath = '/nominals.urdf'

    with open(urdf_filepath, 'r') as infp:
        robot_desc = infp.read()

    return launch.LaunchDescription([
        use_sim_time_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time,
                         'robot_description': robot_desc}],
        ),
    ])
