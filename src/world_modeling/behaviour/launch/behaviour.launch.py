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
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch behaviour node."""
    behaviour_pkg_prefix = get_package_share_directory("behaviour")
    behaviour_param_file = os.path.join(
        behaviour_pkg_prefix, "config", "params.yaml"
    )

    behaviour_param = DeclareLaunchArgument(
        "behaviour_param_file",
        default_value=behaviour_param_file,
        description="Path to config file for behaviour node",
    )

    config_file = LaunchConfiguration("behaviour_param_file")

    behaviour = Node(
        package="behaviour",
        executable="behaviour_node",
        parameters=[config_file],
    )

    return LaunchDescription([behaviour_param, behaviour])
