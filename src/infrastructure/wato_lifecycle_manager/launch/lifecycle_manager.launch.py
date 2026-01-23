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
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    node_names_arg = DeclareLaunchArgument(
        "node_names",
        default_value="[]",
        description="List of lifecycle node names to manage",
    )

    autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically start managed nodes on launch",
    )

    transition_timeout_arg = DeclareLaunchArgument(
        "transition_timeout_s",
        default_value="10.0",
        description="Timeout for lifecycle transitions in seconds",
    )

    bond_timeout_arg = DeclareLaunchArgument(
        "bond_timeout_s",
        default_value="4.0",
        description="Bond heartbeat timeout in seconds",
    )

    bond_enabled_arg = DeclareLaunchArgument(
        "bond_enabled",
        default_value="true",
        description="Enable bond-based crash detection",
    )

    # Lifecycle manager node
    lifecycle_manager_node = Node(
        package="wato_lifecycle_manager",
        executable="wato_lifecycle_manager_node",
        name="lifecycle_manager",
        output="screen",
        parameters=[
            {
                "node_names": LaunchConfiguration("node_names"),
                "autostart": LaunchConfiguration("autostart"),
                "transition_timeout_s": LaunchConfiguration("transition_timeout_s"),
                "bond_timeout_s": LaunchConfiguration("bond_timeout_s"),
                "bond_enabled": LaunchConfiguration("bond_enabled"),
            }
        ],
    )

    return LaunchDescription(
        [
            node_names_arg,
            autostart_arg,
            transition_timeout_arg,
            bond_timeout_arg,
            bond_enabled_arg,
            lifecycle_manager_node,
        ]
    )
