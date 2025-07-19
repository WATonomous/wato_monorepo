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
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    ld = LaunchDescription()
    config = os.path.join(
        get_package_share_directory("lidar_object_detection"),
        "config",
        "eve_config.yaml",
    )

    # nodes
    lidar_object_detection = Node(
        package="lidar_object_detection",
        executable="lidar_object_detection_node",
        name="lidar_object_detection_node",
        parameters=[config],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # finalize
    ld.add_action(lidar_object_detection)

    return ld
