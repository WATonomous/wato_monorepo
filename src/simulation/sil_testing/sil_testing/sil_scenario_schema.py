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
"""Schema + loader for SIL scenario files (pure Python, no ROS dependency).

A single scenario YAML drives the whole SIL run: the simulation side (yaml_scenario
plugin) reads ``map``/``ego``/``actors``/``weather``/``seed`` to build the CARLA world,
and the action side (this module) reads ``route``/``limits``/``duration_s`` to feed the
controller-under-test and check pass/fail. All poses are in the ROS map frame.
"""

from dataclasses import dataclass, field
from typing import Dict, List, Optional

import yaml

DEFAULT_DURATION_S = 30.0
DEFAULT_COMPONENT = "motion_control"


@dataclass
class Waypoint:
    x: float
    y: float


@dataclass
class Route:
    target_speed_mps: float
    waypoints: List[Waypoint]


@dataclass
class Scenario:
    name: str
    component: str
    duration_s: float
    route: Optional[Route]
    limits: Dict[str, float]
    raw: dict = field(default_factory=dict)


def _parse_route(data: Optional[dict]) -> Optional[Route]:
    if not data:
        return None
    if "target_speed_mps" not in data:
        raise ValueError("route is missing required field 'target_speed_mps'")
    raw_wps = data.get("waypoints") or []
    if len(raw_wps) < 2:
        raise ValueError("route requires at least 2 waypoints")
    waypoints = []
    for i, wp in enumerate(raw_wps):
        if "x" not in wp or "y" not in wp:
            raise ValueError(f"route waypoint {i} must define both 'x' and 'y'")
        waypoints.append(Waypoint(float(wp["x"]), float(wp["y"])))
    return Route(float(data["target_speed_mps"]), waypoints)


def _parse_limits(data: Optional[dict]) -> Dict[str, float]:
    limits: Dict[str, float] = {}
    for key, value in (data or {}).items():
        if not (key.startswith("max_") or key.startswith("min_")):
            raise ValueError(
                f"limit '{key}' must start with 'max_' (upper bound) or 'min_' (lower bound)"
            )
        limits[key] = float(value)
    return limits


def parse_scenario(data: dict) -> Scenario:
    """Validate and convert a parsed YAML dict into a Scenario."""
    if not isinstance(data, dict):
        raise ValueError("scenario file must contain a YAML mapping at the top level")
    if "name" not in data:
        raise ValueError("scenario is missing required field 'name'")

    component = data.get("component", DEFAULT_COMPONENT)
    route = _parse_route(data.get("route"))

    if component == "motion_control" and route is None:
        raise ValueError("motion_control scenarios require a 'route' section")

    return Scenario(
        name=str(data["name"]),
        component=str(component),
        duration_s=float(data.get("duration_s", DEFAULT_DURATION_S)),
        route=route,
        limits=_parse_limits(data.get("limits")),
        raw=data,
    )


def load_scenario(path: str) -> Scenario:
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    return parse_scenario(data)
