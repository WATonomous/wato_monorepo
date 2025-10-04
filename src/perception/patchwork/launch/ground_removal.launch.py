# Copyright (c) 2025-present WATonomous. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution(
        [FindPackageShare("patchworkpp"), "config", "params.yaml"]
    )

    # ROS configuration
    cloud_topic_arg = DeclareLaunchArgument(
        "cloud_topic",
        default_value="/LIDAR_TOP",
    )
    pointcloud_topic = LaunchConfiguration("cloud_topic")

    # Patchwork++ ground removal node
    ground_removal_node = Node(
        package="patchworkpp",
        executable="patchworkpp_node",
        name="patchworkpp_node",
        output="screen",
        parameters=[config_file],
        remappings=[
            ("pointcloud_topic", pointcloud_topic),
        ],
    )
    return LaunchDescription([cloud_topic_arg, ground_removal_node])
