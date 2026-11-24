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
"""Resolve and validate data assets shared by the runner and application runtimes."""

import json
import math
import os
import re
from pathlib import Path


def data_name(value, label="asset"):
    if not isinstance(value, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", value):
        raise ValueError(
            f"{label} must be a name containing lowercase letters, digits and underscores"
        )
    return value


def catalog_directory():
    configured = os.environ.get("WATOD_SIM_CATALOG_DIRECTORY")
    if configured:
        return Path(configured)
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("carla_scenarios")) / "config"


def maneuver_directory():
    configured = os.environ.get("WATOD_SIM_MANEUVER_DIRECTORY")
    if configured:
        return Path(configured)
    # Source-only validation needs neither ROS nor installed packages.
    for parent in Path(__file__).resolve().parents:
        candidate = parent / "src/action/fake_planner/maneuvers"
        if candidate.is_dir():
            return candidate
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("fake_planner")) / "maneuvers"


def contained_file(directory, filename):
    root = Path(directory).resolve()
    path = (root / filename).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise ValueError(f"Missing asset inside {root}: {filename}")
    return path


def finite_number(value, label, *, minimum=None, strict=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
    ):
        raise ValueError(f"{label} must be finite")
    if minimum is not None and (value < minimum or (strict and value == minimum)):
        raise ValueError(
            f"{label} must be {'greater than' if strict else 'at least'} {minimum}"
        )
    return value


def load_maneuver(name, directory=None):
    """Accept any valid maneuver file under the configured root, without a name whitelist."""
    data_name(name, "maneuver")
    path = contained_file(directory or maneuver_directory(), name + ".json")

    def invalid_constant(value):
        raise ValueError(f"Non-finite JSON number: {value}")

    document = json.loads(
        path.read_text(encoding="utf-8"), parse_constant=invalid_constant
    )
    if not isinstance(document, dict):
        raise ValueError("Maneuver JSON must be an object")
    for key, default, strict in (
        ("sample_spacing_m", 1.0, True),
        ("default_speed", 2.0, False),
        ("ramp_up_accel", 1.0, False),
        ("brake_decel", 1.0, True),
        ("stop_pad_m", 10.0, False),
    ):
        finite_number(document.get(key, default), key, minimum=0, strict=strict)
    if document.get("ramp_up_accel", 1.0) != 0:
        raise ValueError(
            "Managed pure pursuit requires ramp_up_accel: 0; its timed release ramp handles startup"
        )
    if not isinstance(document.get("closed", False), bool):
        raise ValueError("closed must be a boolean")
    if "start" in document:
        if not isinstance(document["start"], dict):
            raise ValueError("start must be an object")
        for key in ("x", "y", "yaw"):
            finite_number(document["start"].get(key, 0), "start." + key)
    segments = document.get("segments")
    if not isinstance(segments, list) or not segments:
        raise ValueError("Maneuver requires a nonempty segments list")
    for segment in segments:
        if not isinstance(segment, dict) or len(segment) != 1:
            raise ValueError("Each maneuver segment must have one type")
        kind, values = next(iter(segment.items()))
        if kind not in {
            "straight",
            "dwell",
            "shift",
            "slalom",
            "arc",
        } or not isinstance(values, dict):
            raise ValueError("Unsupported maneuver segment")
        positive = {
            "straight": ("length",),
            "dwell": ("length",),
            "shift": ("length",),
            "slalom": ("wavelength", "cycles"),
            "arc": ("radius",),
        }[kind]
        for key in positive:
            finite_number(values.get(key), kind + "." + key, minimum=0, strict=True)
        for key in {
            "shift": ("lateral",),
            "slalom": ("amplitude",),
            "arc": ("angle",),
        }.get(kind, ()):
            finite_number(values.get(key), kind + "." + key)
        if kind == "arc" and (
            values["angle"] == 0 or values.get("dir", "left") not in {"left", "right"}
        ):
            raise ValueError("arc requires a nonzero angle and left/right direction")
        for key in ("speed", "end_speed"):
            if key in values:
                finite_number(values[key], kind + "." + key, minimum=0)
    return path, document


def map_config_file(name):
    """Resolve map configuration as data, including newly added map presets."""
    data_name(name, "map preset")
    configured = os.environ.get("WATOD_SIM_MAP_CONFIG_DIRECTORY")
    if configured:
        directory = Path(configured)
    else:
        directory = None
        for parent in Path(__file__).resolve().parents:
            candidate = (
                parent / "src/world_modeling/world_modeling_bringup/config/map_configs"
            )
            if candidate.is_dir():
                directory = candidate
                break
        if directory is None:
            from ament_index_python.packages import get_package_share_directory

            directory = (
                Path(get_package_share_directory("world_modeling_bringup"))
                / "config/map_configs"
            )
    path = contained_file(directory, name + ".yaml")
    import yaml

    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(document, dict) or not isinstance(
        document.get("/**/world_model"), dict
    ):
        raise ValueError("Map config must configure /**/world_model")
    parameters = document["/**/world_model"].get("ros__parameters")
    if not isinstance(parameters, dict) or not isinstance(
        parameters.get("osm_map_path"), str
    ):
        raise ValueError("Map config must provide osm_map_path")
    return path
