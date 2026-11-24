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

from simulation_runtime.assets import data_name, load_maneuver, map_config_file

MAP_GROUP = {
    "packages": ["world_modeling_bringup", "world_model", "lanelet_markers"],
    "manager": "/world_modeling_lifecycle_manager",
}

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


def launch_groups(preset, inputs):
    """Select lifecycle owners; controller tests can add a map-only world group."""
    if preset == "world_modeling":
        world_mode = inputs.get("world_modeling", "map")
        if world_mode not in {"off", "map", "full"}:
            raise ValueError("world_modeling must be off, map or full")
        return (
            {}
            if world_mode == "off"
            else {
                "world": MAP_GROUP
                if world_mode == "map"
                else PRESETS["autonomy"]["world"]
            }
        )
    if preset not in PRESETS:
        raise ValueError(f"Unsupported preset: {preset}")
    groups = PRESETS[preset]
    if preset == "controller_pure_pursuit":
        world_mode = inputs.get("world_modeling", "map" if "map" in inputs else "off")
        if world_mode not in {"off", "map", "full"}:
            raise ValueError("world_modeling must be off, map or full")
        if world_mode != "off":
            world = MAP_GROUP if world_mode == "map" else PRESETS["autonomy"]["world"]
            return {"world": world, **groups}
    return groups


def command_topic(attempt_id):
    """Map raw attempt IDs injectively to ROS tokens, without requiring host ROS."""
    if not isinstance(attempt_id, str) or not re.fullmatch(
        r"[A-Za-z0-9_]+", attempt_id
    ):
        raise ValueError("Invalid attempt ID")
    # Reserve this prefix for numeric IDs and escape literal uses of it. Simply
    # prefixing digits with "attempt_" aliases e.g. "2" and "attempt_2".
    if attempt_id[0].isdigit() or attempt_id.startswith("_attempt_"):
        attempt_id = f"_attempt_{attempt_id}"
    topic = f"/sim/attempts/{attempt_id}/command"
    # Jazzy's full-topic validator reserves 8 bytes of the 255-byte DDS limit.
    # Keep this helper ROS-free so offline launch validation needs no ROS install.
    if len(topic) > 247:
        raise ValueError("Attempt command topic exceeds the ROS name length limit")
    return topic


def launch_command(role, configuration):
    """Return an argv for one reviewed launch file. Never accept shell commands."""
    preset = configuration["preset"]
    if role not in launch_groups(preset, configuration["inputs"]):
        raise ValueError(f"Unsupported preset/role: {preset}/{role}")
    topic = command_topic(configuration["attempt_id"])
    inputs = configuration["inputs"]
    common = ["use_sim_time:=true", "autostart:=false", "bond_enabled:=false"]
    if role == "world":
        world_map = inputs.get("map")
        data_name(world_map, "map preset")
        configuration_file = map_config_file(world_map)
        return [
            "ros2",
            "launch",
            "world_modeling_bringup",
            "world_modeling_sim.launch.yaml",
            *common,
            "use_eidos:=false",
            f"map_only:={'true' if preset != 'autonomy' and inputs.get('world_modeling', 'map') == 'map' else 'false'}",
            f"map:={world_map}",
            f"map_config_file:={configuration_file}",
        ]
    if preset == "controller_pure_pursuit":
        maneuver = inputs.get("maneuver")
        path, _ = load_maneuver(maneuver)
        return [
            "ros2",
            "launch",
            "fake_planner",
            "fake_planner_sim.launch.yaml",
            *common,
            "frame_id:=map",
            "start_on_activate:=false",
            f"maneuver:={maneuver}",
            f"maneuver_file:={path}",
            f"ackermann_topic:={topic}",
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
