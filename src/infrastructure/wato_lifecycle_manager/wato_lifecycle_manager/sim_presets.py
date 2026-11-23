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
"""Reviewed simulation launch groups, shared by the runner and application runtimes."""

import math
import re

PRESETS = {
    "controller_pure_pursuit": {
        "action": {
            "packages": ["fake_planner", "ackermann_pure_pursuit", "action_bringup"],
            "manager": "/fake_planner_lifecycle_manager",
        },
    },
    "autonomy": {
        "world": {
            "packages": [
                "world_modeling_bringup",
                "world_model",
                "costmap",
                "behaviour",
                "prediction",
            ],
            "manager": "/world_modeling_lifecycle_manager",
        },
        "action": {
            "packages": [
                "action_bringup",
                "lattice_planning",
                "trajectory_planner",
                "ackermann_pure_pursuit",
            ],
            "manager": "/action_lifecycle_manager",
        },
    },
}


def command_topic(attempt_id):
    """Use an attempt ID rather than timestamps to fence commands across clock jumps."""
    if not isinstance(attempt_id, str) or not re.fullmatch(
        r"[A-Za-z0-9_]+", attempt_id
    ):
        raise ValueError("Invalid attempt ID")
    return f"/sim/attempts/{attempt_id}/command"


def launch_command(role, configuration):
    """Return an argv for one reviewed launch file. Never accept shell commands."""
    preset = configuration["preset"]
    if role not in PRESETS.get(preset, {}):
        raise ValueError(f"Unsupported preset/role: {preset}/{role}")
    topic = command_topic(configuration["attempt_id"])
    inputs = configuration["inputs"]
    common = ["use_sim_time:=true", "autostart:=false", "bond_enabled:=false"]
    if preset == "controller_pure_pursuit":
        if inputs.get("maneuver") != "steady_state_slalom":
            raise ValueError("Controller preset requires steady_state_slalom")
        return [
            "ros2",
            "launch",
            "fake_planner",
            "fake_planner_sim.launch.yaml",
            *common,
            "frame_id:=map",
            "start_on_activate:=false",
            "maneuver:=steady_state_slalom",
            f"ackermann_topic:={topic}",
        ]
    if role == "world":
        world_map = inputs.get("map")
        if world_map not in {"oval", "ring_road", "town10hd", "test_track"}:
            raise ValueError("Unsupported map preset")
        return [
            "ros2",
            "launch",
            "world_modeling_bringup",
            "world_modeling_sim.launch.yaml",
            *common,
            "use_eidos:=false",
            f"map:={world_map}",
        ]
    values = {}
    for field in ("stop_distance_m", "max_speed_mps"):
        value = inputs.get(field)
        if (
            isinstance(value, bool)
            or not isinstance(value, (int, float))
            or not math.isfinite(value)
            or value <= 0
        ):
            raise ValueError(f"{field} must be finite and positive")
        values[field] = float(value)
    return [
        "ros2",
        "launch",
        "action_bringup",
        "action_sim.launch.yaml",
        *common,
        f"command_topic:={topic}",
        f"stop_distance:={values['stop_distance_m']}",
        f"max_speed:={values['max_speed_mps']}",
    ]
