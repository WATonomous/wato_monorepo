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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cameras = []
    PANO_DIRS = ["ne", "ee", "se", "ss", "sw", "ww", "nw", "nn"]
    LOWER_DIRS = ["ne", "se", "sw", "nw"]
    cameras.extend("camera_pano_" + dir for dir in PANO_DIRS)
    cameras.extend("camera_lower_" + dir for dir in LOWER_DIRS)

    composable_nodes = []

    for camera in cameras:
        namespace = f"/{camera}"
        debayer_node = ComposableNode(
            name=f"debayer_{camera}",
            namespace=camera,
            package="image_proc",
            plugin="image_proc::DebayerNode",
        )
        composable_nodes.append(debayer_node)

        rectify_node = ComposableNode(
            name=f"rectify_{camera}",
            namespace=namespace,
            package="image_proc",
            plugin="image_proc::RectifyNode",
            remappings=[('image', 'image_color')],

        )
        composable_nodes.append(rectify_node)

    container = ComposableNodeContainer(
        name="image_proc_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=composable_nodes,
    )

    return LaunchDescription([container])
