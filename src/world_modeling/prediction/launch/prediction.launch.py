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
    kalman_filter_pkg_prefix = get_package_share_directory("kalman_filter")
    kalman_filter_param_file = os.path.join(
        kalman_filter_pkg_prefix, "config", "params.yaml"
    )

    kalman_filter_param = DeclareLaunchArgument(
        "kalman_filter_param_file",
        default_value=kalman_filter_param_file,
        description="Path to config file for kalman_filter node",
    )

    kalman_filter_service_node = Node(
        package="kalman_filter",
        executable="kalman_filter_service",
        parameters=[LaunchConfiguration("kalman_filter_param_file")],
    )

    return LaunchDescription([kalman_filter_param, kalman_filter_service_node])
