import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Reference from https://github.com/ros-drivers/flir_camera_driver/blob/humble-devel/spinnaker_camera_driver/launch/driver_node.launch.py

    parameter_file = LaunchConfiguration('parameter_file', default=os.path.join(
        get_package_share_directory('spinnaker_camera_driver'), 'config', 'blackfly_s.yaml'))
    parameter_file_arg = DeclareLaunchArgument('parameter_file', default_value=parameter_file,
                                               description='path to ros parameter definition file (override camera type)')
    serial = LaunchConfiguration('serial', default="'20435008'")
    serial_arg = DeclareLaunchArgument('serial', default_value=serial,
                                       description='FLIR serial number of camera (in quotes!!)')

    camera_node = Node(
        package='spinnaker_camera_driver',
        executable='camera_driver_node',
        output='screen',
        name='flir_camera',
        parameters=[{'parameter_file': parameter_file,
                     'serial_number': [serial]}],
        remappings=[
            ('~/control', '/exposure_control/control'),
        ])

    return LaunchDescription([
        parameter_file_arg,
        serial_arg,
        camera_node
    ])
