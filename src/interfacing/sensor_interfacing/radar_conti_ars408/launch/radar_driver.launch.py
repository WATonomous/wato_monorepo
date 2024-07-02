from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from pathlib import Path
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('radar_conti_ars408'),
        'config',
        'params.yaml'
    )
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config,
        description='Full path to the ROS2 parameters file to use for the radar node'
    )
    lifecycle_nodes = ['radar_node']

    autostart = LaunchConfiguration('autostart')
    autostart_arg = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time',
                                             default_value="False",
                                             description=str("Use sim time argument for whether to force it"))

    namespace = LaunchConfiguration('namespace')
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    # Nodes launching commands

    radar_node = Node(
        package='radar_conti_ars408',
        executable='radar_conti_ars408_composition',
        name='radar_node',
        namespace=LaunchConfiguration('namespace'),
        output='screen',
        parameters=[LaunchConfiguration('params_file')])
    
    # launch socketcan node
    socketcan_dir = get_package_share_directory('ros2_socketcan')
    socketcan_receiver_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                    socketcan_dir + '/launch/socket_can_receiver.launch.py'))

    ld = LaunchDescription()
    ld.add_action(params_file_arg)
    ld.add_action(autostart_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(namespace_arg)
    ld.add_action(radar_node)
    ld.add_action(socketcan_receiver_launch)

    return ld
