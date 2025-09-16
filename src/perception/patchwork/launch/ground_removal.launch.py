from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # tf tree configuration
    base_frame = LaunchConfiguration("base_frame")

    # ROS configuration
    pointcloud_topic = LaunchConfiguration("cloud_topic")
    publish_debug = LaunchConfiguration("publish_debug")
    publish_original = LaunchConfiguration("publish_original")

    # Patchwork++ Ground Removal node
    ground_removal_node = Node(
        package="patchworkpp",
        executable="patchworkpp_node",
        name="patchworkpp_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
        parameters=[
            {
                # ROS node configuration
                "base_frame": base_frame,
                "publish_debug": publish_debug,
                "publish_original": publish_original,
                
                # Patchwork++ configuration
                "sensor_height": 1.88,
                "num_iter": 3,  # Number of iterations for ground plane estimation using PCA.
                "num_lpr": 20,  # Maximum number of points to be selected as lowest points representative.
                "num_min_pts": 0,  # Minimum number of points to be estimated as ground plane in each patch.
                "th_seeds": 0.3,
                # threshold for lowest point representatives using in initial seeds selection of ground points.
                "th_dist": 0.125,  # threshold for thickness of ground.
                "th_seeds_v": 0.25,
                # threshold for lowest point representatives using in initial seeds selection of vertical structural points.
                "th_dist_v": 0.9,  # threshold for thickness of vertical structure.
                "max_range": 80.0,  # max_range of ground estimation area
                "min_range": 1.0,  # min_range of ground estimation area
                "uprightness_thr": 0.101,
                # threshold of uprightness using in Ground Likelihood Estimation(GLE). Please refer paper for more information about GLE.
                "verbose": True,  # display verbose info
            }
        ],
    )


    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("cloud_topic", default_value="/points_raw"),
        DeclareLaunchArgument("publish_debug", default_value="false"),
        DeclareLaunchArgument("publish_original", default_value="false"),

        ground_removal_node
    ])
