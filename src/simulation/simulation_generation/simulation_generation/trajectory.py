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
"""Bounded proposal validation and an ego-frame geometry preview.

The primitive equations and 0.05 m fine sampling follow fake_planner_core.cpp.
This preview omits uniform resampling and speed-profile braking; ROS /sim/reference_path remains
the authoritative anchored path at runtime.
"""

import math

from simulation_runtime.assets import finite_number
from simulation_runtime.scenario_config import keys

from .actors import actor_schema, validate_actors

PARAMETERS = {
    "straight": {"length": (0.5, 300)},
    "dwell": {"length": (0.5, 300)},
    "shift": {"length": (0.5, 300), "lateral": (-15, 15)},
    "slalom": {
        "amplitude": (-5, 5),
        "wavelength": (4, 100),
        "cycles": (0.5, 8),
    },
    "arc": {"radius": (5, 200), "angle": (1, 180)},
}


def tool_schema():
    variants = []
    for kind, bounds in PARAMETERS.items():
        properties = {
            key: {"type": "number", "minimum": low, "maximum": high}
            for key, (low, high) in bounds.items()
        }
        properties.update(
            {
                "speed": {"type": "number", "minimum": 0.5, "maximum": 8},
                "end_speed": {"type": "number", "minimum": 0, "maximum": 8},
            }
        )
        required = list(bounds)
        if kind == "arc":
            properties["dir"] = {"type": "string", "enum": ["left", "right"]}
            required.append("dir")
        variants.append(
            {
                "type": "object",
                "properties": {
                    kind: {
                        "type": "object",
                        "properties": properties,
                        "required": required,
                        "additionalProperties": False,
                    }
                },
                "required": [kind],
                "additionalProperties": False,
            }
        )
    return {
        "type": "function",
        "name": "validate_trajectory",
        "description": "Validate a complete controller path and vehicle actors; fix reported errors and retry.",
        # Optional segment speed overrides use non-strict function calling.
        # Local validation is mandatory, including for schema-conforming calls.
        "strict": False,
        "parameters": {
            "type": "object",
            "properties": {
                "description": {"type": "string"},
                "default_speed": {"type": "number", "minimum": 0.5, "maximum": 8},
                "segments": {
                    "type": "array",
                    "minItems": 1,
                    "maxItems": 32,
                    "items": {"anyOf": variants},
                },
                "actors": actor_schema(),
            },
            "required": ["description", "default_speed", "segments", "actors"],
            "additionalProperties": False,
        },
    }


def bounded(value, low, high, label):
    try:
        finite_number(value, label)
    except OverflowError:
        raise ValueError(f"{label} is outside the finite numeric range") from None
    if not low <= value <= high:
        raise ValueError(f"{label} must be between {low} and {high}")


def validate_proposal(proposal):
    keys(proposal, {"description", "default_speed", "segments", "actors"}, "proposal")
    validate_actors(proposal.get("actors", []))
    if not isinstance(proposal.get("description"), str) or not (
        1 <= len(proposal["description"].strip()) <= 2000
    ):
        raise ValueError("description must contain 1-2000 characters")
    bounded(proposal.get("default_speed"), 0.5, 8, "default_speed")
    segments = proposal.get("segments")
    if not isinstance(segments, list) or not 1 <= len(segments) <= 32:
        raise ValueError("segments must contain 1-32 entries")
    for index, segment in enumerate(segments):
        if not isinstance(segment, dict) or len(segment) != 1:
            raise ValueError(f"segment {index + 1} must contain exactly one primitive")
        kind, values = next(iter(segment.items()))
        if kind not in PARAMETERS:
            raise ValueError(f"Unsupported segment: {kind}")
        bounds = PARAMETERS[kind]
        allowed = set(bounds) | {"speed", "end_speed"}
        if kind == "arc":
            allowed.add("dir")
        keys(values, allowed, f"segment {index + 1} ({kind})")
        for key, (low, high) in bounds.items():
            bounded(values.get(key), low, high, f"{kind}.{key}")
        if kind == "arc" and values.get("dir") not in ("left", "right"):
            raise ValueError("arc.dir must be left or right")
        for key in ("speed", "end_speed"):
            if key in values:
                value = values[key]
                if key == "end_speed" and index == len(segments) - 1 and value == 0:
                    finite_number(value, key)  # Reject bool even though False == 0.
                else:
                    bounded(value, 0.5, 8, f"{kind}.{key}")
    last = segments[-1].get("straight", {})
    if last.get("length", 0) < 5:
        raise ValueError("Finish with a straight segment of at least 5 m")


def sample_geometry(proposal):
    """Match the planner's <= 0.05 m primitive parameter sampling."""
    points = [(0.0, 0.0)]
    x = y = heading = length = 0.0
    for segment in proposal["segments"]:
        kind, values = next(iter(segment.items()))
        if kind == "arc":
            sweep = math.radians(values["angle"])
            distance = values["radius"] * sweep
        elif kind == "slalom":
            distance = values["wavelength"] * values["cycles"]
        else:
            distance = values["length"]
        # Reject oversized paths before allocating samples.
        if distance > 300:
            raise ValueError("A primitive exceeds the 300 m path budget")
        steps = max(1, math.ceil(distance / 0.05))
        for step in range(1, steps + 1):
            t = distance * step / steps
            u, v = t, 0.0
            if kind == "shift":
                v = values["lateral"] * (1 - math.cos(math.pi * t / distance)) / 2
            elif kind == "slalom":
                taper = min(values["wavelength"] / 2, distance / 2)
                envelope = 1.0
                if t < taper:
                    envelope = (1 - math.cos(math.pi * t / taper)) / 2
                elif t > distance - taper:
                    envelope = (1 - math.cos(math.pi * (distance - t) / taper)) / 2
                v = (
                    values["amplitude"]
                    * envelope
                    * math.sin(2 * math.pi * t / values["wavelength"])
                )
            elif kind == "arc":
                sign = 1 if values["dir"] == "left" else -1
                u = values["radius"] * math.sin(t / values["radius"])
                v = sign * values["radius"] * (1 - math.cos(t / values["radius"]))
            point = (
                x + u * math.cos(heading) - v * math.sin(heading),
                y + u * math.sin(heading) + v * math.cos(heading),
            )
            length += math.dist(points[-1], point)
            if length > 300:
                raise ValueError("Sampled path exceeds the 300 m path budget")
            points.append(point)
        x, y = points[-1]
        if kind == "arc":
            heading += sign * sweep
    return points, {
        "authored_length_m": round(length, 3),
        "bounds_m": {
            "min_x": min(p[0] for p in points),
            "max_x": max(p[0] for p in points),
            "min_y": min(p[1] for p in points),
            "max_y": max(p[1] for p in points),
        },
        "end_heading_deg": round(math.degrees(heading), 3),
    }


def preview_svg(points, summary):
    """Fixed markup with numeric coordinates only; model text cannot inject SVG."""
    bounds = summary["bounds_m"]
    x0, x1 = bounds["min_x"] - 5, bounds["max_x"] + 5
    y0, y1 = bounds["min_y"] - 5, bounds["max_y"] + 5
    path = " ".join(f"{x:.3f},{-y:.3f}" for x, y in points)
    return (
        '<svg xmlns="http://www.w3.org/2000/svg" width="1000" height="600" '
        f'viewBox="{x0} {-y1} {x1 - x0} {y1 - y0}">'
        "<title>Ego-relative authored geometry; x forward, y left; metres. "
        "No map projection, stop pad, or driving verification.</title>"
        f'<rect x="{x0}" y="{-y1}" width="{x1 - x0}" height="{y1 - y0}" fill="white"/>'
        f'<polyline points="{path}" fill="none" stroke="#2563eb" '
        'stroke-width="2" vector-effect="non-scaling-stroke"/>'
        '<circle cx="0" cy="0" r="0.6" fill="#16a34a"/>'
        f'<circle cx="{points[-1][0]}" cy="{-points[-1][1]}" r="0.6" fill="#dc2626"/>'
        "</svg>\n"
    )
