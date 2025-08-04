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
    # --- HD Map Node Config ---
    hd_map_pkg_prefix = get_package_share_directory("hd_map")
    hd_map_param_file = os.path.join(hd_map_pkg_prefix, "config", "params.yaml")

    hd_map_param = DeclareLaunchArgument(
        "hd_map_param_file",
        default_value=hd_map_param_file,
        description="Path to config file for hd_map node",
    )

    hd_map_node = Node(
        package="hd_map",
        executable="hd_map_service",
        parameters=[LaunchConfiguration("hd_map_param_file")],
    )

    # --- Localization Node Config ---
    localization_pkg_prefix = get_package_share_directory("localization")
    localization_param_file = os.path.join(
        localization_pkg_prefix, "config", "params.yaml"
    )

    localization_param = DeclareLaunchArgument(
        "localization_param_file",
        default_value=localization_param_file,
        description="Path to config file for localization node",
    )

    localization_node = Node(
        package="localization",
        executable="odom",
        parameters=[LaunchConfiguration("localization_param_file")],
    )

    # -- Occupancy Node Config --
    occupancy_seg_pkg_prefix = get_package_share_directory("occupancy_segmentation")
    occupancy_seg_param_file = os.path.join(
        occupancy_seg_pkg_prefix, "config", "params.yaml"
    )

    occupancy_seg_param = DeclareLaunchArgument(
        "occupancy_segmentation_param_file",
        default_value=occupancy_seg_param_file,
        description="Path to config file for occupancy segmentation node",
    )

    occupancy_seg_node = Node(
        package="occupancy_segmentation",
        executable="occupancy_segmentation_node",
        parameters=[LaunchConfiguration("occupancy_segmentation_param_file")],
    )

    return LaunchDescription(
        [
            hd_map_param,
            localization_param,
            occupancy_seg_param,
            hd_map_node,
            localization_node,
            occupancy_seg_node,
        ]
    )
