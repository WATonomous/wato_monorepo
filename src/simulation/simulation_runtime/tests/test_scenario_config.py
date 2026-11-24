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
"""Reject invalid scenario schemas and malformed data assets."""

import copy
import json
import tempfile
import unittest
from pathlib import Path

from simulation_runtime.assets import load_maneuver
from simulation_runtime.scenario_config import (
    metric_configuration,
    normalize_scenario,
)


def scenario():
    return {
        "version": 2,
        "id": "new_test",
        "trajectory": {"source": "fake_planner", "maneuver": "new_maneuver"},
        "world": {"map": "oval"},
        "completion": {"type": "maneuver_finished"},
        "metrics": {"sample_count": {"min": 2}, "completed": {"equals": True}},
    }


class TestDataAssets(unittest.TestCase):
    def test_bad_json_and_escaping_symlinks_are_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "assets").mkdir()
            for value in (
                "[]",
                '{"segments": []}',
                '{"ramp_up_accel": 0, "segments": [{"straight": {"length": NaN}}]}',
            ):
                (root / "assets/bad.json").write_text(value)
                with self.assertRaises(ValueError):
                    load_maneuver("bad", root / "assets")
            (root / "outside.json").write_text(
                json.dumps(
                    {"ramp_up_accel": 0, "segments": [{"straight": {"length": 12}}]}
                )
            )
            (root / "assets/escape.json").symlink_to(root / "outside.json")
            with self.assertRaises(ValueError):
                load_maneuver("escape", root / "assets")


class TestScenarioConfiguration(unittest.TestCase):
    def test_planner_configuration_requires_a_goal_and_one_trajectory_source(self):
        raw = scenario()
        raw["trajectory"] = {"source": "planner"}
        raw["completion"] = {"type": "goal_reached"}
        with self.assertRaisesRegex(ValueError, "world.goal"):
            normalize_scenario(raw)
        raw["world"]["goal"] = {"x": 20, "y": 30}
        normalize_scenario(raw)
        raw["trajectory"]["maneuver"] = "new_maneuver"
        with self.assertRaisesRegex(ValueError, "two trajectory publishers"):
            normalize_scenario(raw)

    def test_invalid_modes_unknown_metrics_and_inverted_limits_fail_early(self):
        base = scenario()
        for section, field, value in (
            ("world", "modeling", []),
            ("world", "modeling", "everything"),
            ("trajectory", "source", "shell"),
            ("completion", "type", []),
        ):
            changed = copy.deepcopy(base)
            changed[section][field] = value
            with self.subTest(field=field), self.assertRaises(ValueError):
                normalize_scenario(changed)
        for metrics in (
            {"not_implemented": True},
            {"sample_count": {"min": 4, "max": 2}},
            {"completed": {"max": 1}},
            {"speed_mps": {"max": float("nan")}},
        ):
            with self.subTest(metrics=metrics), self.assertRaises(ValueError):
                metric_configuration(metrics)
