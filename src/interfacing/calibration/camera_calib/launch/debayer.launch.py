from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    PANO_DIRS = ["ne", "ee", "se", "ss", "sw", "ww", "nw", "nn"]

    composable_nodes = [
        ComposableNode(
            name="debayer",
            namespace=f"/camera_pano_{dir}",
            package="image_proc",
            plugin="image_proc::DebayerNode",
        ) for dir in PANO_DIRS
    ]

    container = ComposableNodeContainer(
        name="image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes
    )

    return LaunchDescription([container])