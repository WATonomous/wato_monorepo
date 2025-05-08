import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_cameras = LaunchConfiguration('launch_cameras', default=True)
    launch_cameras_arg = DeclareLaunchArgument('launch_cameras', default_value=launch_cameras,
                                               description='Launch Flir Blackfly Cameras')
    launch_lidars = LaunchConfiguration('launch_lidars', default=True)
    launch_lidars_arg = DeclareLaunchArgument('launch_lidars', default_value=launch_lidars,
                                              description='Launch Velodyne LiDAR')
    launch_args = [launch_cameras_arg,
                   launch_lidars_arg]

    sensor_interfacing_include_dir = os.path.join(
        get_package_share_directory('sensor_interfacing'), 'launch', 'include')

    urdf_static_transform_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [sensor_interfacing_include_dir, '/urdf.launch.py'])
    )

    cameras_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [sensor_interfacing_include_dir, '/cameras.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_cameras', 'True')
    )

    lidars_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [sensor_interfacing_include_dir, '/lidars.launch.py']),
        condition=LaunchConfigurationEquals(
            'launch_lidars', 'True')
    )

    return LaunchDescription(launch_args + [
        # urdf_static_transform_publisher_launch, # uncomment once we have a URDF
        cameras_launch,
        lidars_launch
    ])
