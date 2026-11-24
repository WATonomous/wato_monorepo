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
"""Version-two scenario data normalized for the reusable execution profiles."""

import copy

from simulation_runtime.assets import data_name, finite_number

METRIC_UNITS = {
    "lateral_tracking_rmse_m": "m",
    "max_lateral_error_m": "m",
    "completed": "bool",
    "closest_clearance_m": "m",
    "final_clearance_m": "m",
    "distance_progressed_m": "m",
    "stopped_duration_s": "s",
    "sample_count": "samples",
    "goal_distance_m": "m",
    "speed_mps": "m/s",
    "elapsed_s": "s",
    "collision": "bool",
}


def mapping(value, label):
    if not isinstance(value, dict):
        raise ValueError(f"{label} must be a mapping")
    return value


def keys(value, allowed, label):
    mapping(value, label)
    unknown = set(value) - set(allowed)
    if unknown:
        raise ValueError(f"Unknown {label} fields: {', '.join(sorted(unknown))}")


def metric_configuration(metrics):
    mapping(metrics, "metrics")
    if not metrics:
        raise ValueError("metrics must select at least one implemented metric")
    result = {}
    for name, limits in metrics.items():
        if name not in METRIC_UNITS:
            raise ValueError(f"Unknown metric: {name}")
        if limits is False:
            continue
        if limits is True:
            limits = {}
        keys(limits, {"min", "max", "equals"}, "metric " + name)
        if METRIC_UNITS[name] == "bool":
            if set(limits) - {"equals"} or (
                "equals" in limits and not isinstance(limits["equals"], bool)
            ):
                raise ValueError(f"{name} accepts only a boolean equals limit")
        else:
            if "equals" in limits:
                raise ValueError(f"{name} accepts min/max limits")
            for value in limits.values():
                finite_number(value, "metric " + name)
            if limits.get("min", float("-inf")) > limits.get("max", float("inf")):
                raise ValueError(f"Metric {name}: min exceeds max")
        result[name] = dict(limits)
    if not result:
        raise ValueError("metrics cannot all be disabled")
    return result


def metric_failures(values, limits):
    reasons = []
    for name, bounds in limits.items():
        value = values[name]["value"]
        if not bounds:
            continue
        if value is None:
            reasons.append(f"Metric {name} is unavailable")
        elif "equals" in bounds and value != bounds["equals"]:
            reasons.append(f"Metric {name} must equal {bounds['equals']}")
        elif "min" in bounds and value < bounds["min"]:
            reasons.append(f"Metric {name} is below {bounds['min']}")
        elif "max" in bounds and value > bounds["max"]:
            reasons.append(f"Metric {name} exceeds {bounds['max']}")
    return reasons


def normalize_scenario(raw):
    keys(
        raw,
        {
            "version",
            "id",
            "description",
            "trajectory",
            "world",
            "planning",
            "completion",
            "metrics",
            "deadlines",
            "events",
        },
        "scenario",
    )
    trajectory = raw.get("trajectory", {})
    keys(trajectory, {"source", "maneuver"}, "trajectory")
    source = trajectory.get(
        "source", "fake_planner" if "maneuver" in trajectory else "none"
    )
    if not isinstance(source, str) or source not in {"fake_planner", "planner", "none"}:
        raise ValueError("trajectory.source must be fake_planner, planner or none")
    if source != "fake_planner" and "maneuver" in trajectory:
        raise ValueError(
            "A maneuver requires trajectory.source: fake_planner; two trajectory publishers are not allowed"
        )
    world = raw.get("world", {})
    keys(
        world,
        {
            "map",
            "ros_map",
            "modeling",
            "ego",
            "actors",
            "goal",
            "weather",
            "traffic",
            "seed",
            "required_straight_m",
            "require_map_provenance",
        },
        "world",
    )
    world_mode = world.get(
        "modeling", {"planner": "full", "fake_planner": "off", "none": "map"}[source]
    )
    if not isinstance(world_mode, str) or world_mode not in {"off", "map", "full"}:
        raise ValueError("world.modeling must be off, map or full")
    if source == "planner" and world_mode != "full":
        raise ValueError("Planner trajectories require full world modeling")
    playground = data_name(world.get("map"), "world.map")
    ego = world.get("ego", {})
    keys(ego, {"blueprint", "spawn"}, "world.ego")
    setup = {
        key: copy.deepcopy(value)
        for key, value in world.items()
        if key not in {"map", "ros_map", "modeling", "ego"}
    }
    setup["spawn"] = copy.deepcopy(ego.get("spawn", {}))
    if "blueprint" in ego:
        setup["ego_blueprint"] = ego["blueprint"]
    actors = world.get("actors", [])
    if not isinstance(actors, list):
        raise ValueError("world.actors must be a list")
    inputs = {"world_modeling": world_mode}
    if world_mode != "off":
        inputs["map"] = data_name(world.get("ros_map", playground), "world.ros_map")
    if source == "fake_planner":
        inputs["maneuver"] = data_name(
            trajectory.get("maneuver"), "trajectory.maneuver"
        )
    planning = raw.get("planning", {})
    keys(planning, {"stop_distance_m", "max_speed_mps"}, "planning")
    if source == "planner":
        inputs.update(
            {
                "stop_distance_m": planning.get("stop_distance_m", 2.0),
                "max_speed_mps": planning.get("max_speed_mps", 3.0),
            }
        )
        for name in ("stop_distance_m", "max_speed_mps"):
            finite_number(inputs[name], "planning." + name, minimum=0, strict=True)
        if not world.get("goal"):
            raise ValueError("Planner scenarios require world.goal")
    elif planning:
        raise ValueError("planning settings apply only to trajectory.source: planner")
    completion = raw.get("completion", {})
    keys(
        completion,
        {
            "type",
            "actor",
            "clearance_m",
            "speed_mps",
            "hold_s",
            "max_lateral_offset_m",
            "tolerance_m",
            "duration_s",
        },
        "completion",
    )
    kind = completion.get("type")
    if not isinstance(kind, str) or kind not in {
        "maneuver_finished",
        "stopped_behind_actor",
        "goal_reached",
        "duration",
    }:
        raise ValueError("Unsupported completion type")
    completion_fields = {
        "maneuver_finished": {"type"},
        "stopped_behind_actor": {
            "type",
            "actor",
            "clearance_m",
            "speed_mps",
            "hold_s",
            "max_lateral_offset_m",
        },
        "goal_reached": {"type", "tolerance_m", "speed_mps", "hold_s"},
        "duration": {"type", "duration_s"},
    }
    keys(completion, completion_fields[kind], "completion")
    if source == "none" and kind != "duration":
        raise ValueError("World-only scenarios use duration completion")
    if kind == "maneuver_finished" and source != "fake_planner":
        raise ValueError("maneuver_finished requires a fake-planner trajectory")
    thresholds = {}
    if kind == "stopped_behind_actor":
        actor = data_name(completion.get("actor"), "completion.actor")
        setup["evaluation_actor"] = actor
        clearance = completion.get("clearance_m", [2.0, 8.0])
        if not isinstance(clearance, list) or len(clearance) != 2:
            raise ValueError("completion.clearance_m must be [minimum, maximum]")
        for value in clearance:
            finite_number(value, "completion.clearance_m", minimum=0)
        if clearance[0] >= clearance[1]:
            raise ValueError("completion clearance minimum must be below maximum")
        finite_number(
            completion.get("max_lateral_offset_m", 1.5),
            "completion.max_lateral_offset_m",
            minimum=0,
        )
        thresholds.update(
            {
                "min_clearance_m": clearance[0],
                "max_clearance_m": clearance[1],
                "stopped_speed_mps": completion.get("speed_mps", 0.15),
                "stopped_duration_s": completion.get("hold_s", 3.0),
                "max_lateral_offset_m": completion.get("max_lateral_offset_m", 1.5),
                "min_progress_m": 0.0,
            }
        )
        actor_ids = [
            value.get("id")
            for value in world.get("actors", [])
            if isinstance(value, dict)
        ]
        if actor not in actor_ids:
            raise ValueError("completion.actor must name an actor in world.actors")
        target = next(value for value in actors if value.get("id") == actor)
        if target.get("static", True) is not True:
            raise ValueError("stopped_behind_actor requires a static target")
    if kind == "goal_reached" and not world.get("goal"):
        raise ValueError("goal_reached requires world.goal")
    for key, default in (("tolerance_m", 2.0), ("speed_mps", 0.15), ("hold_s", 3.0)):
        if key in completion:
            finite_number(
                completion[key],
                "completion." + key,
                minimum=0,
                strict=key != "speed_mps",
            )
    if kind == "duration":
        finite_number(
            completion.get("duration_s"),
            "completion.duration_s",
            minimum=0,
            strict=True,
        )
    limits = metric_configuration(raw.get("metrics", {}))
    if source != "fake_planner" and set(limits) & {
        "lateral_tracking_rmse_m",
        "max_lateral_error_m",
    }:
        raise ValueError("Tracking error metrics require a fake-planner reference")
    if (
        set(limits) & {"closest_clearance_m", "final_clearance_m", "stopped_duration_s"}
        and kind != "stopped_behind_actor"
    ):
        raise ValueError(
            "Actor clearance/stopping metrics require stopped_behind_actor completion"
        )
    if "goal_distance_m" in limits and not world.get("goal"):
        raise ValueError("goal_distance_m requires world.goal")
    defaults = {
        "readiness_s": 90.0,
        "execution_s": 90.0,
        "stalled_clock_s": 15.0,
        "first_output_s": 3.0,
        "data_freshness_s": 2.0,
    }
    keys(raw.get("deadlines", {}), defaults, "deadlines")
    deadlines = {**defaults, **raw.get("deadlines", {})}
    for key, value in deadlines.items():
        finite_number(value, "deadlines." + key, minimum=0, strict=True)
    if completion.get("duration_s", 0) > deadlines["execution_s"]:
        raise ValueError("Completion duration exceeds execution deadline")
    events = raw.get(
        "events",
        [
            {
                "when": "play",
                "do": "start_maneuver"
                if source == "fake_planner"
                else "begin_evaluation",
            }
        ],
    )
    return {
        "version": 2,
        "id": raw["id"],
        "description": raw.get("description", ""),
        "trajectory_source": source,
        "stack": {
            "fake_planner": "controller_pure_pursuit",
            "planner": "autonomy",
            "none": "world_modeling",
        }[source],
        "playground": playground,
        "setup": setup,
        "inputs": inputs,
        "completion": kind,
        "completion_config": copy.deepcopy(completion),
        "metrics": limits,
        "metric_limits": limits,
        "thresholds": thresholds,
        "deadlines": deadlines,
        "events": events,
        "source_definition": copy.deepcopy(raw),
    }
