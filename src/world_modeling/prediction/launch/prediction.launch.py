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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Launch multi-modal trajectory prediction node."""
    prediction_pkg_prefix = get_package_share_directory("prediction")
    prediction_param_file = os.path.join(
        prediction_pkg_prefix, "config", "params.yaml"
    )

    prediction_param = DeclareLaunchArgument(
        "prediction_param_file",
        default_value=prediction_param_file,
        description="Path to config file for prediction node",
    )

    container = ComposableNodeContainer(
        name="prediction_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="prediction",
                plugin="prediction::PredictionNode",
                name="prediction_node",
                parameters=[LaunchConfiguration("prediction_param_file")],
                remappings=[
                    ("/perception/detections_3D_tracked", "/perception/detections_3D_tracked"),
                    ("/localization/pose", "/localization/pose"),
                    ("/world_modeling/prediction/predicted_paths", "/world_modeling/prediction/predicted_paths"),
                ],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([prediction_param, container])
