from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent,
                            RegisterEventHandler, IncludeLaunchDescription)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.events.lifecycle import ChangeState
from launch.events import matches_action
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition

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

    # config args
    interface_arg = DeclareLaunchArgument('interface', default_value='vcan0')
    enable_can_fd_arg = DeclareLaunchArgument('enable_can_fd', default_value='false')
    interval_sec_arg = DeclareLaunchArgument('interval_sec', default_value='0.01')
    timeout_sec_msg = DeclareLaunchArgument('timeout_sec', default_value='0.01')
    use_bus_time_arg = DeclareLaunchArgument('use_bus_time', default_value='false')
    filters_arg = DeclareLaunchArgument('filters', default_value='0:0')
    auto_configure_arg = DeclareLaunchArgument('auto_configure', default_value='true')
    auto_activate_arg = DeclareLaunchArgument('auto_activate', default_value='true')
    from_can_bus_topic_arg = DeclareLaunchArgument(
        'from_can_bus_topic', default_value='from_can_bus')
    to_can_bus_topic_msg = DeclareLaunchArgument('to_can_bus_topic', default_value='to_can_bus')

    # can sender
    socket_can_sender_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_sender_node_exe',
        name='socket_can_sender',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'enable_can_fd': LaunchConfiguration('enable_can_fd'),
            'timeout_sec':
            LaunchConfiguration('timeout_sec'),
        }],
        remappings=[('to_can_bus', LaunchConfiguration('to_can_bus_topic'))],
        output='screen')

    socket_can_sender_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_sender_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_sender_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_sender_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_sender_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    # can receiver
    socket_can_receiver_node = LifecycleNode(
        package='ros2_socketcan',
        executable='socket_can_receiver_node_exe',
        name='socket_can_receiver',
        namespace=TextSubstitution(text=''),
        parameters=[{
            'interface': LaunchConfiguration('interface'),
            'enable_can_fd': LaunchConfiguration('enable_can_fd'),
            'interval_sec':
            LaunchConfiguration('interval_sec'),
            'filters': LaunchConfiguration('filters'),
            'use_bus_time': LaunchConfiguration('use_bus_time'),
        }],
        remappings=[('from_can_bus', LaunchConfiguration('from_can_bus_topic'))],
        output='screen')

    socket_can_receiver_configure_event_handler = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=socket_can_receiver_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node),
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_configure')),
    )

    socket_can_receiver_activate_event_handler = RegisterEventHandler(
        event_handler=OnStateTransition(
            target_lifecycle_node=socket_can_receiver_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(socket_can_receiver_node),
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    ),
                ),
            ],
        ),
        condition=IfCondition(LaunchConfiguration('auto_activate')),
    )

    # Nodes launching commands

    radar_node = Node(
        package='radar_conti_ars408',
        executable='radar_conti_ars408_composition',
        name='radar_node',
        output='screen',
        parameters=[LaunchConfiguration('params_file')])

    ld = LaunchDescription([
        interface_arg,
        enable_can_fd_arg,
        interval_sec_arg,
        use_bus_time_arg,
        filters_arg,
        auto_configure_arg,
        auto_activate_arg,
        from_can_bus_topic_arg,
        to_can_bus_topic_msg,
        params_file_arg,
        autostart_arg,
        use_sim_time_arg,
        timeout_sec_msg,
        radar_node,
        socket_can_receiver_node,
        socket_can_receiver_configure_event_handler,
        socket_can_receiver_activate_event_handler,
        socket_can_sender_node,
        socket_can_sender_configure_event_handler,
        socket_can_sender_activate_event_handler,
    ])
    # ld.add_action(socketcan_sender_launch)

    return ld
