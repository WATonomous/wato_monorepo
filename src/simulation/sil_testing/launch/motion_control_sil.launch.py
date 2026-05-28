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
"""Motion-control SIL: CARLA bridge + reference feeder + monitor + recorder.

Runs entirely in the simulation image. It brings up the SIL CARLA bridge (via
simulation_bringup/sil_simulation.launch.yaml), synthesizes the reference Trajectory
(trajectory_feeder, replacing the planner), monitors limits live, and records the run.

The controller under test (e.g. pure_pursuit) is an action-module binary and is launched
separately from the action image by the driver (watod sil run); it consumes
/sil/reference_trajectory and publishes to /carla/ackermann_control/command to close the
loop. The scenario_runner ends the run after the scenario duration of sim time, which
tears down this launch and stops the recorder.
"""

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import FrontendLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

SIL_TOPICS = [
    "/clock",
    "/tf",
    "/tf_static",
    "/ego/odom",
    "/sil/reference_trajectory",
    "/carla/ackermann_control/command",
    "/action/is_idle",
    "/carla/detections_3d",
    "/carla/bbox_markers",
    "/carla/scenario_status",
    "/sil/diagnostics",
]


def generate_launch_description():
    default_bag = os.environ.get("SIL_BAG_PATH", "/tmp/sil_bag")
    use_sim_time = {"use_sim_time": True}

    bag_path = LaunchConfiguration("bag_path")
    trajectory_topic = "/sil/reference_trajectory"

    # CARLA bridge (scenario_server -> yaml_scenario, localization, ackermann_control, bbox)
    carla_bridge = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("simulation_bringup"),
                "launch",
                "sil_simulation.launch.yaml",
            )
        )
    )

    trajectory_feeder = Node(
        package="sil_testing",
        executable="trajectory_feeder_node",
        name="trajectory_feeder_node",
        output="screen",
        parameters=[{**use_sim_time, "output_topic": trajectory_topic}],
    )

    monitor = Node(
        package="sil_testing",
        executable="sil_monitor_node",
        name="sil_monitor_node",
        output="screen",
        parameters=[use_sim_time],
    )

    scenario_runner = Node(
        package="sil_testing",
        executable="scenario_runner_node",
        name="scenario_runner_node",
        output="screen",
        parameters=[use_sim_time],
    )

    recorder = ExecuteProcess(
        cmd=["ros2", "bag", "record", "--storage", "mcap", "-o", bag_path] + SIL_TOPICS,
        output="screen",
    )

    # When the run finishes (scenario_runner exits), shut the whole launch down so the
    # recorder flushes and stops cleanly.
    shutdown_on_complete = RegisterEventHandler(
        OnProcessExit(
            target_action=scenario_runner,
            on_exit=[EmitEvent(event=Shutdown(reason="SIL run complete"))],
        )
    )

    return LaunchDescription([
        DeclareLaunchArgument("bag_path", default_value=default_bag,
                              description="Output MCAP bag directory"),
        carla_bridge,
        recorder,
        trajectory_feeder,
        monitor,
        scenario_runner,
        shutdown_on_complete,
    ])
