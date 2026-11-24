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
"""Load named YAML worlds and validate scenario data."""

import copy
import math
import re
from pathlib import Path

import yaml
from simulation_runtime.assets import data_name, finite_number, load_maneuver
from simulation_runtime.scenario_config import normalize_scenario


def merge_settings(base, overrides):
    # Scenario fixtures must not mutate the catalog used by later manual opens.
    result = copy.deepcopy(base)
    for key, value in overrides.items():
        if key == "spawn" and isinstance(value, dict) and "pose" in value:
            result[key] = copy.deepcopy(value)
        elif isinstance(value, dict) and isinstance(result.get(key), dict):
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


def validate_pose(pose, label):
    if not isinstance(pose, dict):
        raise ValueError(f"{label} must be a mapping")
    frame = pose.get("frame", "map")
    if frame == "lane":
        if set(pose) - {"frame", "distance_m", "z_offset_m"}:
            raise ValueError(f"Unknown {label} lane fields")
        finite_number(pose.get("distance_m"), label + ".distance_m", minimum=0)
        finite_number(pose.get("z_offset_m", 0.4), label + ".z_offset_m")
    elif frame == "map":
        if set(pose) - {"frame", "x", "y", "z", "yaw_deg"}:
            raise ValueError(f"Unknown {label} map fields")
        for key in ("x", "y"):
            finite_number(pose.get(key), label + "." + key)
        for key in ("z", "yaw_deg"):
            finite_number(pose.get(key, 0.0), label + "." + key)
    else:
        raise ValueError(f"{label}.frame must be map or lane")


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
    if "pose" in spawn:
        validate_pose(spawn["pose"], "spawn.pose")
        if spawn["pose"].get("frame", "map") != "map":
            raise ValueError("Explicit ego spawn pose must use the map frame")
        if any(key in spawn for key in ("straight_start", "road_id", "lane_id", "s")):
            raise ValueError("Explicit ego pose cannot also select a lane spawn")
    if "goal" in definition:
        validate_pose(definition["goal"], "goal")
    actors = definition.get("actors", [])
    if not isinstance(actors, list):
        raise ValueError("actors must be a list")
    seen = set()
    for actor in actors:
        if not isinstance(actor, dict) or set(actor) - {
            "id",
            "blueprint",
            "pose",
            "static",
        }:
            raise ValueError("Each actor needs id, blueprint, pose and optional static")
        identifier = data_name(actor.get("id"), "actor.id")
        if identifier == "ego_vehicle":
            raise ValueError(
                "Actor id ego_vehicle is reserved for the controlled vehicle"
            )
        if identifier in seen:
            raise ValueError(f"Duplicate actor id: {identifier}")
        seen.add(identifier)
        if not isinstance(actor.get("blueprint"), str) or not re.fullmatch(
            r"vehicle\.[A-Za-z0-9_.]+", actor["blueprint"]
        ):
            raise ValueError("Actor blueprint must name a vehicle")
        if not isinstance(actor.get("static", True), bool):
            raise ValueError("actor.static must be a boolean")
        validate_pose(actor.get("pose"), "actor.pose")
    if (
        definition.get("evaluation_actor")
        and definition["evaluation_actor"] not in seen
    ):
        raise ValueError("Evaluation actor is missing")
    index = spawn.get("index", 0)
    if isinstance(index, bool) or not isinstance(index, int) or index < 0:
        raise ValueError("spawn.index must be a nonnegative integer")
    for key in ("s_margin_m", "z_offset_m", "fallback_z_offset_m"):
        if key in spawn:
            positive_number(spawn[key], "spawn." + key, zero=True)
    positive_number(
        definition.get("required_straight_m", 0.0), "required_straight_m", zero=True
    )
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
    try:
        with path.open(encoding="utf-8") as stream:
            definition = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise ValueError(f"Cannot read definition {path}: {error}") from error
    if not isinstance(definition, dict):
        raise ValueError(f"Expected a YAML mapping in {path}")
    expected_version = 2 if category == "scenarios" else 1
    if (
        type(definition.get("version")) is not int
        or definition["version"] != expected_version
        or definition.get("id") != name
    ):
        raise ValueError(f"Expected version {expected_version} and id {name} in {path}")
    if category == "scenarios":
        definition = normalize_scenario(definition)
        if definition["trajectory_source"] == "fake_planner":
            _, document = load_maneuver(definition["inputs"]["maneuver"])
            if definition["completion"] == "maneuver_finished" and document.get(
                "closed", False
            ):
                raise ValueError("A closed maneuver needs duration or goal completion")
        events = definition["events"]
        if not isinstance(events, list) or not events:
            raise ValueError("events must be a nonempty ordered sequence")
        previous = -1.0
        for event in events:
            if not isinstance(event, dict) or set(event) != {"when", "do"}:
                raise ValueError("Each event needs exactly when/do")
            trigger = event["when"]
            if trigger == "play":
                at = 0.0
            elif isinstance(trigger, dict) and set(trigger) == {"elapsed_s"}:
                at = finite_number(trigger["elapsed_s"], "event.elapsed_s", minimum=0)
            else:
                raise ValueError("Event must trigger at play or elapsed_s")
            if at < previous or at > definition["deadlines"]["execution_s"]:
                raise ValueError(
                    "Event times must be ordered and within the execution deadline"
                )
            previous = at
            if event["do"] not in {"start_maneuver", "begin_evaluation"}:
                raise ValueError("Unsupported event action")
            if (
                event["do"] == "start_maneuver"
                and definition["trajectory_source"] != "fake_planner"
            ):
                raise ValueError("start_maneuver requires a fake-planner trajectory")
        base = load_definition(directory, "playgrounds", definition["playground"])
        if (
            "map" in definition["inputs"]
            and "ros_map" not in definition["source_definition"]["world"]
        ):
            definition["inputs"]["map"] = data_name(
                base.get("ros_map", definition["playground"]), "world.ros_map"
            )
        validate_world(merge_settings(base, definition["setup"]))
    else:
        validate_world(definition)
    return definition


def world_definition(directory, kind, name):
    if kind == "playground":
        return load_definition(directory, "playgrounds", name)
    if kind != "scenario":
        raise ValueError("Expected playground or scenario")
    scenario = load_definition(directory, "scenarios", name)
    base = load_definition(directory, "playgrounds", scenario["playground"])
    return merge_settings(base, scenario.get("setup", {}))
