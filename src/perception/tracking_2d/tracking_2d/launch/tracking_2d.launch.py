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
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    """Launch tracking node."""
    tracking_param_file = PathJoinSubstitution(
        [FindPackageShare("tracking_2d"), "config", "params.yaml"]
    )

    tracking_2d_node = Node(
        package="tracking_2d",
        name="tracking_2d_node",
        executable="tracking_2d_node",
        parameters=[tracking_param_file],
    )

    return LaunchDescription([tracking_2d_node])
