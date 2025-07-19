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
    """Launch depth_estimation node."""
    depth_estimation_pkg_prefix = get_package_share_directory("depth_estimation")
    depth_estimation_param_file = os.path.join(
        depth_estimation_pkg_prefix, "config", "config.yaml"
    )

    depth_estimation_param = DeclareLaunchArgument(
        "depth_estimation_param_file",
        default_value=depth_estimation_param_file,
        description="Path to config file for depth_estimation node",
    )

    depth_estimation_node = Node(
        package="depth_estimation",
        executable="depth_estimation_node",
        parameters=[LaunchConfiguration("depth_estimation_param_file")],
    )

    return LaunchDescription([depth_estimation_param, depth_estimation_node])
