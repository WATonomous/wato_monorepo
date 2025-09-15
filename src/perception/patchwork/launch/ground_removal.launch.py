from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import (LaunchConfiguration, PathJoinSubstitution,
                                  PythonExpression)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time", default="true")

    # tf tree configuration
    base_frame = LaunchConfiguration("base_frame", default="base_link")

    # ROS configuration
    pointcloud_topic = LaunchConfiguration("cloud_topic")
    visualize = LaunchConfiguration("visualize", default="true")
    publish_debug = LaunchConfiguration("publish_debug", default="false")
    publish_original = LaunchConfiguration("publish_original", default="false")

    # Optional ros bag play
    bagfile = LaunchConfiguration("bagfile", default="")

    # Patchwork++ Ground Removal node
    ground_removal_node = Node(
        package="patchworkpp",
        executable="patchworkpp_ground_removal_node",
        name="patchworkpp_ground_removal_node",
        output="screen",
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
        parameters=[
            {
                # ROS node configuration
                "base_frame": base_frame,
                "use_sim_time": use_sim_time,
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

    # Optional bagfile playback
    bagfile_play = ExecuteProcess(
        cmd=["ros2", "bag", "play", bagfile],
        output="screen",
        condition=IfCondition(PythonExpression(["'", bagfile, "' != ''"])),
    )
