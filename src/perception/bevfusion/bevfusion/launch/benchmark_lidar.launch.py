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

"""Replay-only camera batching and BEVFusion, with explicit lidar backend selection."""

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    cameras = [f"camera_pano_{name}" for name in ("nn", "ne", "nw", "ss", "sw", "se")]
    camera_sync = ComposableNodeContainer(
        name="benchmark_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=[
            ComposableNode(
                package="camera_sync",
                plugin="camera_sync::MultiCameraSyncNode",
                name="multi_camera_sync",
                parameters=[
                    {
                        "use_sim_time": True,
                        "camera_topics": [
                            f"/{name}/image_rect_compressed" for name in cameras
                        ],
                        "camera_info_topics": [
                            f"/{name}/camera_info" for name in cameras
                        ],
                        "camera_names": cameras,
                        "use_compressed": True,
                        "sync_tolerance_ms": 500.0,
                        "queue_size": 10,
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        ],
    )
    bevfusion = Node(
        package="bevfusion",
        executable="bevfusion_node",
        name="bevfusion_node",
        output="screen",
        parameters=[
            LaunchConfiguration("config_file"),
            {
                "use_sim_time": True,
                "use_nitros_images": False,
                "use_raw_images": False,
                "lidar_processing_backend": LaunchConfiguration("backend"),
                "model_dir": LaunchConfiguration("model_dir"),
                "benchmark_csv_path": LaunchConfiguration("csv_path"),
            },
        ],
        remappings=[
            ("camera_info", "/multi_camera_sync/multi_camera_info"),
            ("output_detections", "/benchmark/detections"),
            ("output_markers", "/benchmark/markers"),
        ],
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", choices=["cpu", "cuda"]),
            DeclareLaunchArgument("model_dir"),
            DeclareLaunchArgument("csv_path"),
            DeclareLaunchArgument(
                "config_file",
                default_value=get_package_share_directory("bevfusion")
                + "/config/params.yaml",
            ),
            camera_sync,
            bevfusion,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=bevfusion,
                    on_exit=[EmitEvent(event=Shutdown(reason="BEVFusion exited"))],
                )
            ),
        ]
    )
