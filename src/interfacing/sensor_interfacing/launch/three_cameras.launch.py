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
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    camera_params = {
        "debug": False,
        "dump_node_map": False,
        "pixel_format": "BayerRG8",
        "exposure_auto": "On",
        "exposure_time": 9000,
        "gain_auto": "Off",
        "gain": 0.0,
        "trigger_mode": "Off",
        "frame_rate_enable": True,
        "frame_rate": 60.0,
        "frame_rate_auto": "Off",
        "buffer_queue_size": 1000,
        "stream_buffer_handling_mode": "NewestOnly",
        "multicast_monitor_mode": False,
        "gev_scps_packet_size": 9000,
    }

    cams = [
        ("flir_camera_0", "18542606"),
        ("flir_camera_1", "17453304"),
        ("flir_camera_2", "17453317"),
    ]

    parameter_file = PathJoinSubstitution(
        [FindPackageShare("spinnaker_camera_driver"), "config", "blackfly.yaml"]
    )

    composables = []
    for name, serial in cams:
        composables.append(
            ComposableNode(
                package="spinnaker_camera_driver",
                plugin="spinnaker_camera_driver::CameraDriver",
                name="camera",
                namespace=name,
                parameters=[
                    camera_params,
                    {"serial_number": serial, "parameter_file": parameter_file},
                ],
                remappings=[("~/control", "/exposure_control/control")],
                extra_arguments=[{"use_intra_process_comms": True}],
            )
        )

    container = ComposableNodeContainer(
        name="multi_camera_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        output="screen",
        composable_node_descriptions=composables,
    )

    return LaunchDescription([container])