from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("patchworkpp"), "config", "params.yaml"]
    )

    # tf tree configuration
    base_frame = LaunchConfiguration("base_frame")

    # ROS configuration
    pointcloud_topic = LaunchConfiguration("cloud_topic")
    publish_debug = LaunchConfiguration("publish_debug")
    publish_original = LaunchConfiguration("publish_original")

    # Patchwork++ ground removal node
    ground_removal_node = Node(
        package="patchworkpp",
        executable="patchworkpp_node",
        name="patchworkpp_node",
        output="screen",
        parameters=[
            config_file,
            {
                "base_frame": base_frame,
                "publish_debug": publish_debug,
                "publish_original": publish_original,
                "sensor_height": 1.88,
                "num_iter": 3,
                "num_lpr": 20,
                "num_min_pts": 0,
                "th_seeds": 0.3,
                "th_dist": 0.125,
                "th_seeds_v": 0.25,
                "th_dist_v": 0.9,
                "max_range": 80.0,
                "min_range": 1.0,
                "uprightness_thr": 0.101,
                "verbose": True,
            }
        ],
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("cloud_topic", default_value="/LIDAR_TOP"),
        DeclareLaunchArgument("publish_debug", default_value="false"),
        DeclareLaunchArgument("publish_original", default_value="false"),
        ground_removal_node,
    ])
