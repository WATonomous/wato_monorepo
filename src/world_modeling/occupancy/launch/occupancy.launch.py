from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

port = DeclareLaunchArgument(
    "port", default_value=TextSubstitution(text="8765")
)
address = DeclareLaunchArgument(
    "address", default_value=TextSubstitution(text="0.0.0.0")
)
tls = DeclareLaunchArgument(
    "tls", default_value=TextSubstitution(text="false")
)
certfile = DeclareLaunchArgument(
    "certfile", default_value=TextSubstitution(text="")
)
keyfile = DeclareLaunchArgument(
    "keyfile", default_value=TextSubstitution(text="")
)
topic_whitelist = DeclareLaunchArgument(
    "topic_whitelist", default_value=TextSubstitution(text="['.*']")
)
param_whitelist = DeclareLaunchArgument(
    "param_whitelist", default_value=TextSubstitution(text="['.*']")
)
service_whitelist = DeclareLaunchArgument(
    "service_whitelist", default_value=TextSubstitution(text="['.*']")
)
client_topic_whitelist = DeclareLaunchArgument(
    "client_topic_whitelist", default_value=TextSubstitution(text="['.*']")
)
max_qos_depth = DeclareLaunchArgument(
    "max_qos_depth", default_value=TextSubstitution(text="10")
)
num_threads = DeclareLaunchArgument(
    "num_threads", default_value=TextSubstitution(text="0")
)
send_buffer_limit = DeclareLaunchArgument(
    "send_buffer_limit", default_value=TextSubstitution(text="10000000")
)
use_sim_time = DeclareLaunchArgument(
    "use_sim_time", default_value=TextSubstitution(text="false")
)
capabilities = DeclareLaunchArgument(
    "capabilities", default_value=TextSubstitution(text="[clientPublish,parameters,parametersSubscribe,services,connectionGraph]")
)


def generate_launch_description():
    """Launch sample node."""
    pkg_prefix = get_package_share_directory('occupancy')
    sample_param_file = os.path.join(
        pkg_prefix, 'config', 'sample_params.yaml')

    sample_param = DeclareLaunchArgument(
        'sample_param_file',
        default_value=sample_param_file,
        description='Path to config file for sample node'
    )

    sample_node = Node(
        package='occupancy',
        executable='sample_node',
        parameters=[LaunchConfiguration('sample_param_file')],
    )

    voxel_segmentation_node = Node(
        package='occupancy',
        executable='voxel_segmentation_node.py'
    )

    return LaunchDescription([
        sample_param,
        sample_node,
        voxel_segmentation_node,
        port,
        address,
        tls,
        certfile,
        keyfile,
        topic_whitelist,
        param_whitelist,
        service_whitelist,
        client_topic_whitelist,
        max_qos_depth,
        num_threads,
        send_buffer_limit,
        use_sim_time,
        capabilities, 
        Node(
            package='foxglove_bridge',
            executable='foxglove_bridge',
            parameters=[{
                "port": LaunchConfiguration('port'),
                "address": LaunchConfiguration('address'),
                "tls": LaunchConfiguration('tls'),
                "certfile": LaunchConfiguration('certfile'),
                "keyfile": LaunchConfiguration('keyfile'),
                "topic_whitelist": LaunchConfiguration('topic_whitelist'),
                "service_whitelist": LaunchConfiguration('service_whitelist'),
                "param_whitelist": LaunchConfiguration('param_whitelist'),
                "client_topic_whitelist": LaunchConfiguration('client_topic_whitelist'),
                "max_qos_depth": LaunchConfiguration('max_qos_depth'),
                "num_threads": LaunchConfiguration('num_threads'),
                "send_buffer_limit": LaunchConfiguration('send_buffer_limit'),
                "use_sim_time": LaunchConfiguration('use_sim_time'),
                "capabilities": LaunchConfiguration('capabilities'),
            }]
        )
    ])
