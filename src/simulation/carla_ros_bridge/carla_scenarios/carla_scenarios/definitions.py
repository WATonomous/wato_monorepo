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
"""Installed YAML catalogs and explicit compatibility for legacy world selectors."""

import copy
import math
import re
from pathlib import Path

import yaml

LEGACY_PLAYGROUNDS = {
    "carla_scenarios.scenarios.default_scenario": "town10hd_default",
    "carla_scenarios.scenarios.light_traffic_scenario": "town10hd_light",
    "carla_scenarios.scenarios.heavy_traffic_scenario": "town10hd_heavy",
    "carla_scenarios.scenarios.empty_scenario": "empty",
    "carla_scenarios.scenarios.custom_scenario": "custom",
    "carla_scenarios.scenarios.oval_scenario": "oval",
    "carla_scenarios.scenarios.ring_road_scenario": "ring_road",
}
SETUP_KEYS = {
    "spawn",
    "traffic",
    "weather",
    "seed",
    "ego_blueprint",
    "parked",
    "required_straight_m",
    "goal_distance_m",
    "require_map_provenance",
}


def playground_name(name):
    if name in LEGACY_PLAYGROUNDS:
        return LEGACY_PLAYGROUNDS[name]
    return LEGACY_PLAYGROUNDS.get("carla_scenarios.scenarios." + name, name)


def merge_settings(base, overrides):
    # Scenario fixtures must not mutate the catalog used by later manual opens.
    result = copy.deepcopy(base)
    for key, value in overrides.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = merge_settings(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def positive_number(value, label, *, zero=False):
    if (
        isinstance(value, bool)
        or not isinstance(value, (int, float))
        or not math.isfinite(value)
        or value < 0
        or (not zero and value == 0)
    ):
        raise ValueError(
            f"{label} must be finite and {'nonnegative' if zero else 'positive'}"
        )


def validate_world(definition):
    world_map = definition.get("map", {})
    if not isinstance(world_map, dict) or set(world_map) not in ({"town"}, {"xodr"}):
        raise ValueError("map must select exactly one town or xodr")
    name = next(iter(world_map.values()))
    if not isinstance(name, str) or not name:
        raise ValueError("map name must be a nonempty string")
    if "xodr" in world_map and (Path(name).name != name or not name.endswith(".xodr")):
        raise ValueError("map.xodr must be an OpenDRIVE filename")
    seed = definition.get("seed", 42)
    if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
        raise ValueError("seed must be a nonnegative integer")
    spawn = definition.get("spawn", {})
    if not isinstance(spawn, dict):
        raise ValueError("spawn must be a mapping")
    index = spawn.get("index", 0)
    if isinstance(index, bool) or not isinstance(index, int) or index < 0:
        raise ValueError("spawn.index must be a nonnegative integer")
    for key in ("s_margin_m", "z_offset_m", "fallback_z_offset_m"):
        if key in spawn:
            positive_number(spawn[key], "spawn." + key, zero=True)
    for key in ("required_straight_m", "goal_distance_m"):
        positive_number(definition.get(key, 0.0), key, zero=True)
    if definition.get("parked"):
        positive_number(definition["parked"].get("distance_m"), "parked.distance_m")
    traffic = definition.get("traffic", {})
    if not isinstance(traffic, dict):
        raise ValueError("traffic must be a mapping")
    for key in ("vehicles", "pedestrians"):
        count = traffic.get(key, 0)
        if isinstance(count, bool) or not isinstance(count, int) or count < 0:
            raise ValueError(f"traffic.{key} must be a nonnegative integer")
    crossing = traffic.get("pedestrian_crossing_factor", 0.0)
    positive_number(crossing, "traffic.pedestrian_crossing_factor", zero=True)
    if crossing > 1.0:
        raise ValueError("pedestrian crossing factor must be at most 1")
    props = traffic.get("props", {})
    if not isinstance(props, dict):
        raise ValueError("traffic.props must be a mapping")
    count = props.get("count", 0)
    if isinstance(count, bool) or not isinstance(count, int) or count < 0:
        raise ValueError("traffic.props.count must be a nonnegative integer")
    if count and (
        not isinstance(props.get("blueprints"), list)
        or not props["blueprints"]
        or not all(isinstance(item, str) and item for item in props["blueprints"])
    ):
        raise ValueError("traffic.props.blueprints must list blueprint filters")
    if not isinstance(definition.get("weather", "ClearNoon"), str):
        raise ValueError("weather must name a CARLA weather preset")


def load_definition(directory, category, name):
    """Resolve only installed, named definitions; reject malformed manifests early."""
    if not re.fullmatch(r"[a-z][a-z0-9_]*", name):
        raise ValueError(f"Invalid {category} name {name!r}; use watod sim list.")
    path = directory / category / f"{name}.yaml"
    if not path.is_file():
        raise ValueError(f"Missing {category} definition: {path}; use watod sim list.")
    with path.open(encoding="utf-8") as stream:
        definition = yaml.safe_load(stream)
    if not isinstance(definition, dict):
        raise ValueError(f"Expected a YAML mapping in {path}")
    if definition.get("version") != 1 or definition.get("id") != name:
        raise ValueError(f"Expected version: 1 and id: {name} in {path}")
    if category == "scenarios":
        stack = definition.get("stack")
        completion = definition.get("completion")
        inputs = definition.get("inputs", {})
        if not isinstance(inputs, dict):
            raise ValueError(f"inputs must be a mapping in {path}")
        if (stack, completion) not in {
            ("controller_pure_pursuit", "maneuver_finished"),
            ("autonomy", "stopped_behind_actor"),
        }:
            raise ValueError(f"Unsupported stack/completion in {path}")
        if (
            stack == "controller_pure_pursuit"
            and inputs.get("maneuver") != "steady_state_slalom"
        ):
            raise ValueError(
                "V1 controller preset requires shipped steady_state_slalom"
            )
        events = definition.get("events")
        if not isinstance(events, list) or not events:
            raise ValueError(f"events must be a nonempty ordered YAML sequence: {path}")
        previous = -1.0
        for event in events:
            if not isinstance(event, dict) or set(event) != {"when", "do"}:
                raise ValueError(f"Each event needs exactly when/do: {path}")
            trigger = event["when"]
            if trigger == "play":
                at = 0.0
            elif isinstance(trigger, dict) and set(trigger) == {"elapsed_s"}:
                at = trigger["elapsed_s"]
            else:
                raise ValueError(f"Unsupported event trigger: {trigger}")
            if (
                isinstance(at, bool)
                or not isinstance(at, (int, float))
                or not math.isfinite(at)
                or at < 0.0
                or at < previous
            ):
                raise ValueError(
                    "Event timestamps must be finite, nonnegative and ordered"
                )
            previous = at
            if event["do"] not in {"start_maneuver", "begin_evaluation"}:
                raise ValueError(f"Unsupported reviewed action: {event['do']}")
            if event["do"] == "start_maneuver" and stack != "controller_pure_pursuit":
                raise ValueError("start_maneuver requires controller_pure_pursuit")
        for section in ("deadlines", "thresholds"):
            values = definition.get(section)
            if not isinstance(values, dict) or not values:
                raise ValueError(f"Missing {section}: {path}")
            for key, value in values.items():
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    raise ValueError(f"{section}.{key} must be numeric")
                if not math.isfinite(value) or value <= 0:
                    raise ValueError(f"{section}.{key} must be finite and positive")
        required_deadlines = {
            "readiness_s",
            "execution_s",
            "stalled_clock_s",
            "first_output_s",
            "data_freshness_s",
        }
        required_thresholds = {"min_progress_m", "min_samples"}
        required_thresholds |= (
            {"lateral_tracking_rmse_m", "max_lateral_error_m"}
            if stack == "controller_pure_pursuit"
            else {
                "min_clearance_m",
                "max_clearance_m",
                "stopped_speed_mps",
                "stopped_duration_s",
            }
        )
        if not required_deadlines <= definition["deadlines"].keys():
            raise ValueError(f"Missing deadline fields in {path}")
        if not required_thresholds <= definition["thresholds"].keys():
            raise ValueError(f"Missing metric thresholds in {path}")
        if previous > definition["deadlines"]["execution_s"]:
            raise ValueError("An event falls after the execution deadline")
        if (
            stack == "autonomy"
            and definition["thresholds"]["min_clearance_m"]
            >= definition["thresholds"]["max_clearance_m"]
        ):
            raise ValueError("min_clearance_m must be below max_clearance_m")
        world = load_definition(
            directory, "playgrounds", definition.get("playground", "")
        )
        overrides = definition.get("setup", {})
        if not isinstance(overrides, dict) or set(overrides) - SETUP_KEYS:
            raise ValueError(f"Unsupported world setup overrides in {path}")
        validate_world(merge_settings(world, overrides))
        if stack == "autonomy":
            if not re.fullmatch(r"[a-z][a-z0-9_]*", inputs.get("map", "")):
                raise ValueError("autonomy inputs.map must select a named map preset")
            for key in ("stop_distance_m", "max_speed_mps"):
                positive_number(inputs.get(key), f"inputs.{key}")
    else:
        validate_world(definition)
    return definition


def world_definition(directory, kind, name):
    if kind == "playground":
        return load_definition(directory, "playgrounds", playground_name(name))
    if kind != "scenario":
        raise ValueError("Expected playground or scenario")
    scenario = load_definition(directory, "scenarios", name)
    base = load_definition(directory, "playgrounds", scenario["playground"])
    return merge_settings(base, scenario.get("setup", {}))
