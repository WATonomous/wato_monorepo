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
"""Vehicle actor proposals using the existing configured-world pose formats."""

import re

from carla_scenarios.definitions import validate_pose
from simulation_runtime.assets import data_name
from simulation_runtime.scenario_config import keys

MAX_ACTORS = 16
MAX_LANE_DISTANCE_M = 300


def actor_schema():
    return {
        "type": "array",
        "maxItems": MAX_ACTORS,
        "description": "Named vehicles to spawn; use [] when no actors are requested.",
        "items": {
            "type": "object",
            "properties": {
                "id": {"type": "string", "pattern": "^[a-z][a-z0-9_]*$"},
                "blueprint": {
                    "type": "string",
                    "pattern": r"^vehicle\.[A-Za-z0-9_.]+$",
                    "description": "CARLA vehicle blueprint, e.g. vehicle.mini.cooper",
                },
                "static": {
                    "type": "boolean",
                    "default": True,
                    "description": "True freezes a parked vehicle; false enables physics, not autopilot.",
                },
                "pose": {
                    "anyOf": [
                        {
                            "type": "object",
                            "properties": {
                                "frame": {"type": "string", "enum": ["lane"]},
                                "distance_m": {
                                    "type": "number",
                                    "minimum": 0,
                                    "maximum": MAX_LANE_DISTANCE_M,
                                },
                                "z_offset_m": {"type": "number", "default": 0.4},
                            },
                            "required": ["frame", "distance_m"],
                            "additionalProperties": False,
                        },
                        {
                            "type": "object",
                            "properties": {
                                "frame": {"type": "string", "enum": ["map"]},
                                "x": {"type": "number"},
                                "y": {"type": "number"},
                                "z": {"type": "number", "default": 0.4},
                                "yaw_deg": {"type": "number", "default": 0.0},
                            },
                            "required": ["frame", "x", "y"],
                            "additionalProperties": False,
                        },
                    ]
                },
            },
            "required": ["id", "blueprint", "pose"],
            "additionalProperties": False,
        },
    }


def validate_actors(actors):
    """Reject unsupported actor data before the full scenario-loader validation."""
    if not isinstance(actors, list) or len(actors) > MAX_ACTORS:
        raise ValueError(f"actors must be a list with at most {MAX_ACTORS} vehicles")
    seen = set()
    for index, actor in enumerate(actors):
        label = f"actors[{index}]"
        keys(actor, {"id", "blueprint", "static", "pose"}, label)
        identifier = data_name(actor.get("id"), label + ".id")
        if identifier == "ego_vehicle":
            raise ValueError(
                "Actor id ego_vehicle is reserved for the controlled vehicle"
            )
        if identifier in seen:
            raise ValueError(f"Duplicate actor id: {identifier}")
        seen.add(identifier)
        blueprint = actor.get("blueprint")
        if not isinstance(blueprint, str) or not re.fullmatch(
            r"vehicle\.[A-Za-z0-9_.]+", blueprint
        ):
            raise ValueError(f"{label}.blueprint must name a vehicle")
        if not isinstance(actor.get("static", True), bool):
            raise ValueError(f"{label}.static must be a boolean")
        pose = actor.get("pose")
        if not isinstance(pose, dict) or pose.get("frame") not in ("lane", "map"):
            raise ValueError(f"{label}.pose.frame must be lane or map")
        try:
            validate_pose(pose, label + ".pose")
        except OverflowError:
            raise ValueError(
                f"{label}.pose is outside the finite numeric range"
            ) from None
        if pose["frame"] == "lane" and pose["distance_m"] > MAX_LANE_DISTANCE_M:
            raise ValueError(
                f"{label}.pose.distance_m must be at most {MAX_LANE_DISTANCE_M}"
            )
