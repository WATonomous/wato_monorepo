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
"""Generator contracts: data validation, repair loop, and non-overwriting publish."""

import json
import math
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
import urllib.error
from pathlib import Path
from unittest.mock import Mock, patch

ROOT = Path(__file__).resolve().parents[4]
for directory in (
    "src/simulation/simulation_generation",
    "src/simulation/simulation_runtime",
    "src/simulation/carla_ros_bridge/carla_scenarios",
):
    sys.path.insert(0, str(ROOT / directory))

import yaml  # noqa: E402
from carla_scenarios.definitions import load_definition  # noqa: E402
from simulation_generation.cli import publish  # noqa: E402
from simulation_generation.generator import (  # noqa: E402
    Workspace,
    generate,
    read_json,
    request_response,
)
from simulation_generation.trajectory import (  # noqa: E402
    sample_geometry,
    validate_proposal,
)
from simulation_runtime.sim_presets import launch_groups  # noqa: E402


def proposal():
    return {
        "description": "An ego-relative controller path",
        "default_speed": 2.0,
        "segments": [
            {"straight": {"length": 8}},
            {"shift": {"length": 12, "lateral": 1}},
            {"dwell": {"length": 3}},
            {"straight": {"length": 8, "end_speed": 0}},
        ],
    }


def response(candidate):
    return {
        "status": "completed",
        "output": [
            {"type": "reasoning", "id": "reasoning-id", "summary": []},
            {
                "type": "function_call",
                "name": "validate_trajectory",
                "call_id": "call-id",
                "arguments": json.dumps(candidate),
            },
        ],
    }


class WorkspaceTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.catalog = self.root / "catalog"
        self.maneuvers = self.root / "maneuvers"
        self.map_configs = self.root / "map_configs"
        for directory in (
            self.catalog / "scenarios",
            self.catalog / "playgrounds",
            self.maneuvers,
            self.map_configs,
            self.root / "maps/xodr",
            self.root / "maps/osm",
        ):
            directory.mkdir(parents=True)
        (self.catalog / "playgrounds/test_map.yaml").write_text(
            yaml.safe_dump(
                {"version": 1, "id": "test_map", "map": {"xodr": "test.xodr"}}
            )
        )
        (self.map_configs / "test_map.yaml").write_text(
            yaml.safe_dump(
                {
                    "/**/world_model": {
                        "ros__parameters": {
                            "osm_map_path": "/opt/watonomous/maps/osm/test.osm"
                        }
                    }
                }
            )
        )
        (self.root / "maps/xodr/test.xodr").write_text("<OpenDRIVE/>")
        (self.root / "maps/osm/test.osm").write_text("<osm/>")
        environment = patch.dict(
            os.environ,
            {
                "WATOD_SIM_CATALOG_DIRECTORY": str(self.catalog),
                "WATOD_SIM_MANEUVER_DIRECTORY": str(self.maneuvers),
                "WATOD_SIM_MAP_CONFIG_DIRECTORY": str(self.map_configs),
                "OPENAI_API_KEY": "unit-test-key",
            },
        )
        environment.start()
        self.addCleanup(environment.stop)
        self.workspace = Workspace(self.root, "novel_controller_path", "test_map")

    def test_published_data_loads_through_runtime_and_infers_controller_groups(self):
        scenario, maneuver, _, _ = self.workspace.validate(proposal())
        publish(self.workspace, json.dumps(maneuver), yaml.safe_dump(scenario))
        loaded = load_definition(self.catalog, "scenarios", self.workspace.name)
        self.assertEqual(loaded["trajectory_source"], "fake_planner")
        self.assertEqual(
            set(launch_groups(loaded["stack"], loaded["inputs"])), {"action", "world"}
        )
        self.assertNotIn("start", maneuver)
        self.assertEqual(maneuver["ramp_up_accel"], 0)

    def test_validation_does_not_mutate_catalog_or_environment(self):
        previous = dict(os.environ)
        self.workspace.validate(proposal())
        self.assertEqual(dict(os.environ), previous)
        self.assertFalse(any(self.maneuvers.iterdir()))
        self.assertFalse(any((self.catalog / "scenarios").iterdir()))

    def test_collision_rolls_back_only_new_maneuver(self):
        existing = self.workspace.targets[1]
        existing.write_text("user authored YAML")
        with self.assertRaises(FileExistsError):
            publish(self.workspace, "{}", "new YAML")
        self.assertFalse(self.workspace.targets[0].exists())
        self.assertEqual(existing.read_text(), "user authored YAML")
        with self.assertRaisesRegex(ValueError, "overwrite"):
            self.workspace.check_targets()

    def test_broken_symlink_and_invalid_names_cannot_be_overwritten(self):
        self.workspace.targets[0].symlink_to(self.root / "missing")
        with self.assertRaisesRegex(ValueError, "overwrite"):
            self.workspace.check_targets()
        for name in ("../escape", "HasCaps", "a/b", ""):
            with self.subTest(name=name), self.assertRaises(ValueError):
                Workspace(self.root, name, "test_map")

    def test_missing_map_asset_fails_before_generation(self):
        (self.root / "maps/osm/test.osm").unlink()
        with self.assertRaisesRegex(ValueError, "Missing ROS map"):
            Workspace(self.root, "another_path", "test_map")

    def test_model_repairs_validation_error_with_reasoning_and_tool_feedback(self):
        invalid = proposal()
        invalid["segments"][0]["straight"]["length"] = -1
        request = Mock(side_effect=[response(invalid), response(proposal())])
        candidate, report = generate(
            self.workspace, "custom swerve", "test-model", request=request
        )
        self.assertEqual(candidate, proposal())
        self.assertEqual([item["valid"] for item in report["attempts"]], [False, True])
        second = request.call_args_list[1].args[0]
        self.assertIn(response(invalid)["output"][0], second["input"])
        feedback = next(
            item
            for item in second["input"]
            if item.get("type") == "function_call_output"
        )
        self.assertFalse(json.loads(feedback["output"])["valid"])
        self.assertFalse(second["store"])
        self.assertFalse(any(self.maneuvers.iterdir()))

    def test_model_loop_is_bounded_and_never_publishes_invalid_data(self):
        request = Mock(return_value=response({"segments": []}))
        with self.assertRaisesRegex(ValueError, "after 2 model calls"):
            generate(
                self.workspace, "path", "test-model", max_rounds=2, request=request
            )
        self.assertEqual(request.call_count, 2)
        self.assertFalse(any(self.maneuvers.iterdir()))

    def test_unsupported_request_explanation_is_returned_without_installing(self):
        request = Mock(
            return_value={
                "output": [
                    {
                        "type": "message",
                        "content": [
                            {
                                "type": "output_text",
                                "text": "Actor placement is unsupported",
                            }
                        ],
                    }
                ]
            }
        )
        with self.assertRaisesRegex(ValueError, "Actor placement is unsupported"):
            generate(self.workspace, "add an obstacle", "test-model", request=request)
        self.assertFalse(any(self.maneuvers.iterdir()))

    def test_arbitrary_tool_calls_are_rejected(self):
        result = response(proposal())
        result["output"][1]["name"] = "execute_python"
        with self.assertRaisesRegex(ValueError, "Unexpected model tool"):
            generate(
                self.workspace, "path", "test-model", request=Mock(return_value=result)
            )

    def test_missing_api_key_prevents_network_calls(self):
        request = Mock()
        with patch.dict(os.environ, {"OPENAI_API_KEY": ""}):
            with self.assertRaisesRegex(ValueError, "OPENAI_API_KEY"):
                generate(self.workspace, "path", "test-model", request=request)
        request.assert_not_called()

    def test_host_cli_offline_install_dry_run_and_duplicate_rejection(self):
        # The host entrypoint uses its checkout's map mount. This independent
        # catalog uses a CARLA town and an explicit temporary OSM asset instead.
        (self.catalog / "playgrounds/test_map.yaml").write_text(
            yaml.safe_dump(
                {"version": 1, "id": "test_map", "map": {"town": "Town10HD_Opt"}}
            )
        )
        (self.map_configs / "test_map.yaml").write_text(
            yaml.safe_dump(
                {
                    "/**/world_model": {
                        "ros__parameters": {
                            "osm_map_path": str(self.root / "maps/osm/test.osm")
                        }
                    }
                }
            )
        )
        candidate = self.root / "proposal.json"
        candidate.write_text(json.dumps(proposal()))
        command = [
            sys.executable,
            str(ROOT / "watod_scripts/watod-sim.py"),
            "generate",
            "scenario",
            "--map",
            "test_map",
            "--name",
            "cli_path",
            "--candidate",
            str(candidate),
        ]
        dry = subprocess.run(command + ["--dry-run"], capture_output=True, text=True)
        self.assertEqual(dry.returncode, 0, dry.stderr)
        self.clean_artifacts(dry.stdout)
        self.assertFalse((self.maneuvers / "cli_path.json").exists())
        installed = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(installed.returncode, 0, installed.stderr)
        self.clean_artifacts(installed.stdout)
        self.assertTrue((self.catalog / "scenarios/cli_path.yaml").is_file())
        again = subprocess.run(command, capture_output=True, text=True)
        self.assertEqual(again.returncode, 2)
        self.assertIn("overwrite", again.stderr)

    def clean_artifacts(self, stdout):
        for line in stdout.splitlines():
            if line.startswith("Preview and generation record: "):
                path = Path(line.split(": ", 1)[1])
                self.assertEqual(path.parent, ROOT / ".cache/watod-sim/generated")
                self.addCleanup(shutil.rmtree, path)


class ProposalTests(unittest.TestCase):
    def test_malformed_or_unbounded_inputs_are_rejected(self):
        cases = []
        for key, value in (
            ("start", {}),
            ("default_speed", math.nan),
            ("default_speed", True),
            ("default_speed", 0),
            ("default_speed", 10**1000),
        ):
            candidate = proposal()
            candidate[key] = value
            cases.append(candidate)
        for segment in (
            {"arc": {"radius": 10, "angle": -90, "dir": "left"}},
            {"straight": {"length": 10, "unexpected": True}},
            {"straight": {"length": 10, "end_speed": 0}},
            {"shift": {"length": 10, "lateral": math.inf}},
            {"straight": {"length": 300}},
            {"slalom": {"amplitude": 5, "wavelength": 100, "cycles": 8}},
        ):
            candidate = proposal()
            candidate["segments"].insert(0, segment)
            cases.append(candidate)
        candidate = proposal()
        candidate["segments"][-1] = {"straight": {"length": 2}}
        cases.append(candidate)
        for candidate in cases:
            with self.subTest(candidate=candidate), self.assertRaises(ValueError):
                validate_proposal(candidate)
                sample_geometry(candidate)

    def test_arc_rotation_and_shift_continuity_use_ros_left_positive_frame(self):
        for direction, sign in (("left", 1), ("right", -1)):
            candidate = proposal()
            candidate["segments"] = [
                {"arc": {"radius": 10, "angle": 90, "dir": direction}},
                {"straight": {"length": 5}},
            ]
            validate_proposal(candidate)
            points, _ = sample_geometry(candidate)
            self.assertAlmostEqual(points[-1][0], 10)
            self.assertAlmostEqual(points[-1][1], sign * 15)
        points, _ = sample_geometry(proposal())
        self.assertAlmostEqual(points[-1][1], 1)

    def test_nonfinite_json_is_rejected_at_the_boundary(self):
        for constant in ("NaN", "Infinity", "-Infinity"):
            with self.assertRaises(ValueError):
                read_json('{"default_speed":' + constant + "}")

    def test_transport_uses_responses_api_and_redacts_http_errors(self):
        response_handle = Mock()
        response_handle.read.return_value = b'{"status":"completed","output":[]}'
        response_handle.__enter__ = Mock(return_value=response_handle)
        response_handle.__exit__ = Mock(return_value=False)
        with patch("urllib.request.urlopen", return_value=response_handle) as urlopen:
            request_response({"model": "test-model"}, "private-key")
        request = urlopen.call_args.args[0]
        self.assertEqual(request.full_url, "https://api.openai.com/v1/responses")
        failure = urllib.error.HTTPError(request.full_url, 401, "private-key", {}, None)
        with patch("urllib.request.urlopen", side_effect=failure):
            with self.assertRaisesRegex(ValueError, "HTTP 401") as caught:
                request_response({}, "private-key")
        self.assertNotIn("private-key", str(caught.exception))

    def test_incomplete_api_response_cannot_be_used_as_a_candidate(self):
        response_handle = Mock()
        response_handle.read.return_value = b'{"status":"incomplete","output":[]}'
        response_handle.__enter__ = Mock(return_value=response_handle)
        response_handle.__exit__ = Mock(return_value=False)
        with patch("urllib.request.urlopen", return_value=response_handle):
            with self.assertRaisesRegex(ValueError, "did not complete"):
                request_response({}, "unit-test-key")


if __name__ == "__main__":
    unittest.main()
