from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Create composable nodes for multiple cameras with corrected debayer setup and multithreading."""
    # Common camera parameters
    camera_params = {
        'debug': False,
        'dump_node_map': False,
        'pixel_format': 'BayerRG8',
        'exposure_auto': 'Off',
        'exposure_time': 9000,
        'gain_auto': 'Off',
        'gain': 0.0,
        'trigger_mode': 'Off',
        'frame_rate_auto': 'Off',
        'frame_rate_enable': True,
        'frame_rate': 30.0,
        'buffer_queue_size': 2000,
        'stream_buffer_handling_mode': 'NewestOnly',
        'multicast_monitor_mode': False,
        'gev_scps_packet_size': 1000,
    }

    # Camera details
    cams = [
        ('flir_camera_0', '18542606'),
        ('flir_camera_1', '17453304'),
        ('flir_camera_2', '17453317'),
    ]

    parameter_file = PathJoinSubstitution(
        [FindPackageShare('spinnaker_camera_driver'), 'config', 'blackfly.yaml']
    )

    composables = []
    for name, serial in cams:
        # Camera driver node in its own namespace
        composables.append(
            ComposableNode(
                package='spinnaker_camera_driver',
                plugin='spinnaker_camera_driver::CameraDriver',
                name='camera',
                namespace=name,
                parameters=[camera_params, {'serial_number': serial,
                                            'parameter_file': parameter_file}],
                remappings=[('~/control', '/exposure_control/control')],
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )
        # # Debayer in the same namespace, auto-subscribes to /<name>/image_raw
        composables.append(
            ComposableNode(
                package='image_proc',
                plugin='image_proc::DebayerNode',
                name='debayer',
                namespace=name,
                extra_arguments=[{'use_intra_process_comms': True}],
            )
        )

    # Multi-threaded container for parallel processing
    container = ComposableNodeContainer(
        name='multi_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='screen',
        composable_node_descriptions=composables,
    )

    return LaunchDescription([container])
