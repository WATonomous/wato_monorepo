from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    composable_nodes = [
        ComposableNode(
            name="debayer",
            namespace="/camera_pano_ww",
            package="image_proc",
            plugin="image_proc::DebayerNode",
        )
    ]

    container = ComposableNodeContainer(
        name="image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes
    )

    return LaunchDescription([container])