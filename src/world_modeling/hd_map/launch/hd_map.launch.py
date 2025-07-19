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
    """Launch sample node."""
    hd_map_pkg_prefix = get_package_share_directory("hd_map")
    hd_map_param_file = os.path.join(hd_map_pkg_prefix, "config", "params.yaml")

    hd_map_param = DeclareLaunchArgument(
        "hd_map_param_file",
        default_value=hd_map_param_file,
        description="Path to config file for hd_map node",
    )

    hd_map_service_node = Node(
        package="hd_map",
        executable="hd_map_service",
        parameters=[LaunchConfiguration("hd_map_param_file")],
    )

    return LaunchDescription([hd_map_param, hd_map_service_node])
