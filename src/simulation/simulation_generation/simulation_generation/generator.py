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
"""Curated context, isolated validation, and a bounded Responses API tool loop."""

import contextlib
import copy
import hashlib
import json
import os
import shutil
import tempfile
import urllib.error
import urllib.request
from pathlib import Path

import yaml
from carla_scenarios.definitions import load_definition
from simulation_runtime.assets import (
    contained_file,
    data_name,
    load_maneuver,
    map_config_file,
)

from .trajectory import sample_geometry, tool_schema, validate_proposal


def read_json(text):
    def nonfinite(value):
        raise ValueError(f"Non-finite JSON number: {value}")

    return json.loads(text, parse_constant=nonfinite)


@contextlib.contextmanager
def data_environment(maneuvers, map_configs):
    values = {
        "WATOD_SIM_MANEUVER_DIRECTORY": str(maneuvers),
        "WATOD_SIM_MAP_CONFIG_DIRECTORY": str(map_configs),
    }
    previous = {key: os.environ.get(key) for key in values}
    os.environ.update(values)
    try:
        yield
    finally:
        for key, value in previous.items():
            if value is None:
                os.environ.pop(key, None)
            else:
                os.environ[key] = value


class Workspace:
    def __init__(self, root, name, map_name):
        self.root = Path(root).resolve()
        self.name = data_name(name, "scenario name")
        self.map_name = data_name(map_name, "map")

        def directory(variable, default):
            path = Path(os.environ.get(variable, default))
            path = (self.root / path).resolve()
            if not path.is_dir():
                raise ValueError(f"{variable} directory does not exist: {path}")
            return path

        self.catalog = directory(
            "WATOD_SIM_CATALOG_DIRECTORY",
            "src/simulation/carla_ros_bridge/carla_scenarios/config",
        )
        self.maneuvers = directory(
            "WATOD_SIM_MANEUVER_DIRECTORY", "src/action/fake_planner/maneuvers"
        )
        self.map_configs = directory(
            "WATOD_SIM_MAP_CONFIG_DIRECTORY",
            "src/world_modeling/world_modeling_bringup/config/map_configs",
        )
        self.playground = load_definition(self.catalog, "playgrounds", self.map_name)
        self.ros_map = data_name(self.playground.get("ros_map", self.map_name))
        with data_environment(self.maneuvers, self.map_configs):
            self.map_config = map_config_file(self.ros_map)
        if "xodr" in self.playground["map"]:
            contained_file(self.root / "maps/xodr", self.playground["map"]["xodr"])
        parameters = yaml.safe_load(self.map_config.read_text())["/**/world_model"][
            "ros__parameters"
        ]
        osm = Path(parameters["osm_map_path"])
        container_root = Path("/opt/watonomous/maps")
        if osm.is_relative_to(container_root):
            osm = self.root / "maps" / osm.relative_to(container_root)
        elif not osm.is_absolute():
            osm = self.root / osm
        if not osm.is_file():
            raise ValueError(f"Missing ROS map asset: {osm}")

    @property
    def targets(self):
        return (
            self.maneuvers / f"{self.name}.json",
            self.catalog / "scenarios" / f"{self.name}.yaml",
        )

    def check_targets(self):
        for target in self.targets:
            if target.exists() or target.is_symlink():
                raise ValueError(f"Refusing to overwrite {target}; choose a new --name")

    def context(self):
        # Small, fixed example set; the selected playground is always current.
        examples = []
        for name in (
            "controller_snake",
            "brake_in_curve",
            "sine_with_dwell",
            "parked_car_straight",
        ):
            path = self.catalog / "scenarios" / f"{name}.yaml"
            if not path.is_file():
                continue
            if path.stat().st_size > 24000:
                raise ValueError(f"Context example is too large: {path}")
            raw = yaml.safe_load(path.read_text(encoding="utf-8"))
            maneuver = raw.get("trajectory", {}).get("maneuver")
            example = {"scenario": raw}
            if maneuver:
                with data_environment(self.maneuvers, self.map_configs):
                    _, example["maneuver"] = load_maneuver(maneuver)
            examples.append(example)
        result = {
            "selected_map": self.map_name,
            "playground": self.playground,
            "examples": examples,
        }
        if len(json.dumps(result)) > 64000:
            raise ValueError("Catalog context exceeds 64 KB")
        return result

    def validate(self, proposal):
        validate_proposal(proposal)
        points, summary = sample_geometry(proposal)
        actors = copy.deepcopy(proposal.get("actors", []))
        maneuver = {
            "name": self.name,
            "description": proposal["description"],
            "sample_spacing_m": 0.5,
            "default_speed": proposal["default_speed"],
            "ramp_up_accel": 0.0,
            "brake_decel": 1.0,
            "stop_pad_m": 10.0,
            "closed": False,
            "segments": copy.deepcopy(proposal["segments"]),
        }
        # A straight lane is only a spawn/runout heuristic, never road-fit proof.
        runout = max(20.0, summary["bounds_m"]["max_x"] + 15.0)
        for actor in actors:
            if actor["pose"]["frame"] == "lane":
                runout = max(runout, actor["pose"]["distance_m"] + 5.0)
        scenario = {
            "version": 2,
            "id": self.name,
            "description": proposal["description"],
            "trajectory": {"source": "fake_planner", "maneuver": self.name},
            "world": {
                "map": self.map_name,
                "ros_map": self.ros_map,
                "modeling": "map",
                "ego": {
                    "spawn": {
                        "straight_start": True,
                        "s_margin_m": 10.0,
                        "z_offset_m": 0.4,
                    }
                },
                "required_straight_m": round(runout, 3),
                "traffic": {"vehicles": 0, "pedestrians": 0, "props": {"count": 0}},
                "actors": actors,
            },
            "completion": {"type": "maneuver_finished"},
            "metrics": {
                "lateral_tracking_rmse_m": True,
                "max_lateral_error_m": True,
                "distance_progressed_m": True,
                "completed": {"equals": True},
                "collision": {"equals": False},
            },
            # Budget for minimum accepted moving speed, release, and final braking.
            "deadlines": {
                "execution_s": round(summary["authored_length_m"] / 0.5 + 60, 1)
            },
        }
        with tempfile.TemporaryDirectory(prefix="watod-sim-validate-") as temporary:
            stage = Path(temporary)
            shutil.copytree(self.catalog / "playgrounds", stage / "playgrounds")
            (stage / "scenarios").mkdir()
            (stage / f"{self.name}.json").write_text(
                json.dumps(maneuver), encoding="utf-8"
            )
            (stage / "scenarios" / f"{self.name}.yaml").write_text(
                yaml.safe_dump(scenario, sort_keys=False), encoding="utf-8"
            )
            with data_environment(stage, self.map_configs):
                load_maneuver(self.name, stage)
                load_definition(stage, "scenarios", self.name)
        return scenario, maneuver, points, summary


def request_response(payload, key):
    request = urllib.request.Request(
        "https://api.openai.com/v1/responses",
        data=json.dumps(payload, allow_nan=False).encode(),
        headers={"Authorization": "Bearer " + key, "Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urllib.request.urlopen(request, timeout=90) as response:
            # Bound remote data before decoding. Do not log raw responses or headers.
            raw = response.read(1_000_001)
            if len(raw) > 1_000_000:
                raise ValueError("OpenAI response exceeds 1 MB")
            result = read_json(raw)
    except urllib.error.HTTPError as error:
        raise ValueError(
            f"OpenAI request failed (HTTP {error.code}); check API key, model access, and quota"
        ) from None
    except (urllib.error.URLError, TimeoutError) as error:
        raise ValueError(
            f"OpenAI connection failed ({type(error).__name__}); retry generation"
        ) from None
    if not isinstance(result, dict) or result.get("status") != "completed":
        raise ValueError("OpenAI response did not complete; no candidate was installed")
    return result


def generate(workspace, prompt, model, *, max_rounds=4, request=None):
    """The only model action is proposing data to a local, read-only validator."""
    key = os.environ.get("OPENAI_API_KEY", "")
    if not key:
        raise ValueError(
            "Set OPENAI_API_KEY on the host, or use --candidate for offline validation"
        )
    request = request or (lambda payload: request_response(payload, key))
    system = Path(__file__).with_name("system_prompt.txt").read_text(encoding="utf-8")
    context = workspace.context()
    history = [
        {
            "role": "user",
            "content": json.dumps({"prompt": prompt, "catalog_context": context}),
        }
    ]
    attempts = []
    for round_number in range(1, max_rounds + 1):
        result = request(
            {
                "model": model,
                "instructions": system,
                "input": history,
                "tools": [tool_schema()],
                "tool_choice": "auto",
                "parallel_tool_calls": False,
                "max_output_tokens": 4000,
                "include": ["reasoning.encrypted_content"],
                "store": False,
            }
        )
        output = result.get("output", [])
        calls = [item for item in output if item.get("type") == "function_call"]
        if not calls:
            messages = [
                part.get("text", part.get("refusal", ""))
                for item in output
                if item.get("type") == "message"
                for part in item.get("content", [])
            ]
            explanation = (
                " ".join(messages)[:2000] or "No validation tool call returned"
            )
            raise ValueError(
                f"Generation stopped without a valid trajectory: {explanation}"
            )
        if len(calls) != 1 or calls[0].get("name") != "validate_trajectory":
            raise ValueError("Unexpected model tool calls; no files were installed")
        call = calls[0]
        history.extend(output)  # Preserve reasoning items together with tool calls.
        try:
            proposal = read_json(call["arguments"])
            validated = workspace.validate(proposal)
        except (ValueError, TypeError, KeyError, OverflowError) as error:
            diagnostic = str(error)[:2000]
            attempts.append(
                {"round": round_number, "valid": False, "error": diagnostic}
            )
            history.append(
                {
                    "type": "function_call_output",
                    "call_id": call["call_id"],
                    "output": json.dumps({"valid": False, "error": diagnostic}),
                }
            )
            continue
        attempts.append({"round": round_number, "valid": True})
        return proposal, {
            "model": model,
            "system_prompt_sha256": hashlib.sha256(system.encode()).hexdigest(),
            "context": context,
            "attempts": attempts,
            "geometry": validated[3],
        }
    raise ValueError(
        f"No valid trajectory after {max_rounds} model calls: {attempts[-1]['error']}"
    )
