from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch PointCloudFilter node."""
    radar_pointcloud_filter_node = Node(
        package='radar_pointcloud_filter',
        executable='radar_pointcloud_filter_node',
    )

    return LaunchDescription([
        radar_pointcloud_filter_node
    ])
