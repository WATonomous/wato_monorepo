from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import LaunchConfigurationEquals


def generate_launch_description():
    """Launch ARSPointCloudFilter node."""
    ars_pointcloud_filter_param = DeclareLaunchArgument(
        'filter_mode', default_value='ars')

    ars_pointcloud_filter_node = Node(
        LaunchConfiguration('filter_mode'),
        condition=LaunchConfigurationEquals('filter_mode', 'ars'),
        package='ars_pointcloud_filter',
        executable='ars_pointcloud_filter_node',
        parameters=[
            {'vrel_rad': -99999.99},
            {'el_ang': -99999.99},
            {'rcs0': -99999.99},
            {'snr': -99999.99},
            {'range': -99999.99},
            {'az_ang0': -99999.99},
            {'scan_mode': 'near'}
        ]
    )
    return LaunchDescription([
        ars_pointcloud_filter_param,
        ars_pointcloud_filter_node
    ])
