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
    """Launch the full perception stack."""

    camera_detection_pkg = get_package_share_directory("camera_object_detection")
    camera_detection_param_file = os.path.join(
        camera_detection_pkg, "config", "combined_config.yaml"
    )
    camera_detection_param = DeclareLaunchArgument(
        "camera_object_detection_param_file",
        default_value=camera_detection_param_file,
        description="Path to config file for camera object detection nodes",
    )
    camera_object_detection_node = Node(
        package="camera_object_detection",
        executable="camera_object_detection_node",
        name="camera_object_detection_node",
        parameters=[LaunchConfiguration("camera_object_detection_param_file")],
    )

    depth_estimation_pkg = get_package_share_directory("depth_estimation")
    depth_estimation_param_file = os.path.join(
        depth_estimation_pkg, "config", "config.yaml"
    )
    depth_estimation_param = DeclareLaunchArgument(
        "depth_estimation_param_file",
        default_value=depth_estimation_param_file,
        description="Path to config file for depth estimation node",
    )
    depth_estimation_node = Node(
        package="depth_estimation",
        executable="depth_estimation_node",
        name="depth_estimation_node",
        parameters=[LaunchConfiguration("depth_estimation_param_file")],
    )

    patchwork_pkg = get_package_share_directory("patchworkpp")
    patchwork_param_file = os.path.join(patchwork_pkg, "config", "params.yaml")
    patchwork_param = DeclareLaunchArgument(
        "patchwork_param_file",
        default_value=patchwork_param_file,
        description="Path to config file for patchwork ground removal node",
    )
    patchwork_cloud_topic = DeclareLaunchArgument(
        "patchwork_cloud_topic",
        default_value="/lidar_cc",
        description="Input point cloud topic consumed by Patchwork++",
    )
    patchwork_ground_topic = DeclareLaunchArgument(
        "patchwork_ground_topic",
        default_value="/patchworkpp/ground_cloud",
        description="Ground points output topic from Patchwork++",
    )
    patchwork_non_ground_topic = DeclareLaunchArgument(
        "patchwork_non_ground_topic",
        default_value="/patchworkpp/non_ground_cloud",
        description="Non-ground points output topic from Patchwork++",
    )
    patchwork_node = Node(
        package="patchworkpp",
        executable="patchworkpp_node",
        name="patchworkpp_node",
        parameters=[LaunchConfiguration("patchwork_param_file")],
        remappings=[
            ("input_cloud", LaunchConfiguration("patchwork_cloud_topic")),
            ("ground_cloud", LaunchConfiguration("patchwork_ground_topic")),
            ("non_ground_cloud", LaunchConfiguration("patchwork_non_ground_topic")),
        ],
    )

    spatial_association_pkg = get_package_share_directory("spatial_association")
    spatial_association_param_file = os.path.join(
        spatial_association_pkg, "config", "params.yaml"
    )
    spatial_association_param = DeclareLaunchArgument(
        "spatial_association_param_file",
        default_value=spatial_association_param_file,
        description="Path to config file for spatial_association fusion node",
    )
    spatial_association_node = Node(
        package="spatial_association",
        executable="spatial_association_node",
        name="spatial_association_node",
        parameters=[LaunchConfiguration("spatial_association_param_file")],
    )

    # Attribute Assigner: enriches 2D detections with semantic attributes
    attribute_assigner_pkg = get_package_share_directory("attribute_assigner")
    attribute_assigner_param_file = os.path.join(
        attribute_assigner_pkg, "config", "params.yaml"
    )
    attribute_assigner_param = DeclareLaunchArgument(
        "attribute_assigner_param_file",
        default_value=attribute_assigner_param_file,
        description="Path to config file for attribute assigner node",
    )
    attribute_assigner_image_topic = DeclareLaunchArgument(
        "attribute_assigner_image_topic",
        default_value="/camera/image",
        description="Camera image topic for attribute assigner (same frame as detections)",
    )
    attribute_assigner_input_topic = DeclareLaunchArgument(
        "attribute_assigner_input_topic",
        default_value="/camera_object_detections",
        description="Input detection topic consumed by the attribute assigner",
    )
    attribute_assigner_output_topic = DeclareLaunchArgument(
        "attribute_assigner_output_topic",
        default_value="/enriched_detections",
        description="Output enriched detection topic from the attribute assigner",
    )
    attribute_assigner_node = Node(
        package="attribute_assigner",
        executable="attribute_assigner_node",
        name="attribute_assigner_node",
        parameters=[LaunchConfiguration("attribute_assigner_param_file")],
        remappings=[
            ("input_image", LaunchConfiguration("attribute_assigner_image_topic")),
            ("input_detections", LaunchConfiguration("attribute_assigner_input_topic")),
            (
                "output_detections",
                LaunchConfiguration("attribute_assigner_output_topic"),
            ),
        ],
    )

    return LaunchDescription(
        [
            camera_detection_param,
            depth_estimation_param,
            patchwork_param,
            patchwork_cloud_topic,
            patchwork_ground_topic,
            patchwork_non_ground_topic,
            spatial_association_param,
            attribute_assigner_param,
            attribute_assigner_image_topic,
            attribute_assigner_input_topic,
            attribute_assigner_output_topic,
            camera_object_detection_node,
            depth_estimation_node,
            patchwork_node,
            spatial_association_node,
            attribute_assigner_node,
        ]
    )
