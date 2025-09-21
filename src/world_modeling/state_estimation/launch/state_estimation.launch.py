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
    """Launch state_estimation node."""
    state_estimation_pkg_prefix = get_package_share_directory("state_estimation")
    state_estimation_param_file = os.path.join(
        state_estimation_pkg_prefix, "config", "params.yaml"
    )

    state_estimation_param = DeclareLaunchArgument(
        "state_estimation_param_file",
        default_value=state_estimation_param_file,
        description="Path to config file for state_estimation node",
    )

    config_file = LaunchConfiguration("state_estimation_param_file")

    wheel_odometry = Node(
        package="state_estimation",
        executable="wheel_odometry",
        parameters=[config_file],
    )

    return LaunchDescription([state_estimation_param, wheel_odometry])
