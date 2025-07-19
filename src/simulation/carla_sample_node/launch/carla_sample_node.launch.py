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
import launch_ros.actions

# For creating launch arguments
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    publish_autopilot_arg = DeclareLaunchArgument(
        "publish_autopilot", default_value="False"
    )

    return LaunchDescription(
        [
            publish_autopilot_arg,
            launch_ros.actions.Node(
                namespace="carla_sample_node",
                package="carla_sample_node",
                executable="carla_sample_node",
                parameters=[
                    {"publish_autopilot": LaunchConfiguration("publish_autopilot")}
                ],
                output="screen",
            ),
        ]
    )
