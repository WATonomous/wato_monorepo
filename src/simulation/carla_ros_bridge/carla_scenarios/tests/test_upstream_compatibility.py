# Copyright (c) 2026-present WATonomous. All rights reserved.
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
"""Generated-track compatibility without ROS or CARLA processes."""

import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock, patch

from carla_scenarios.scenario_server_node import ScenarioServerNode
from carla_scenarios.scenarios.oval_track_scenario import OvalTrackScenario


class TestGeneratedTrackCompatibility(unittest.TestCase):
    def test_legacy_selectors_keep_the_original_plugin_and_asset_paths(self):
        for selector in (
            "oval_track_scenario",
            "carla_scenarios.scenarios.oval_track_scenario",
        ):
            node = SimpleNamespace(get_logger=Mock())
            with (
                patch.object(
                    Path, "read_text", return_value="<OpenDRIVE><road/></OpenDRIVE>"
                ) as read,
                patch.object(
                    OvalTrackScenario, "_read_spawn_pose", return_value=(1.0, 2.0, 0.0)
                ),
            ):
                world = ScenarioServerNode._read_playground(node, selector)
            self.assertIsInstance(world, OvalTrackScenario)
            self.assertEqual(
                world.XODR_PATH, "/opt/watonomous/maps/osm/oval_track_4_lane.xodr"
            )
            self.assertEqual(
                world.SPAWN_PATH, "/opt/watonomous/maps/osm/oval_track_spawn.json"
            )
            read.assert_called_once_with(encoding="utf-8")
            self.assertIsNone(world.client)
            self.assertIsNone(world.world)

    def test_invalid_spawn_sidecar_fails_before_world_mutation(self):
        node = SimpleNamespace(get_logger=Mock())
        with (
            patch.object(
                Path, "read_text", return_value="<OpenDRIVE><road/></OpenDRIVE>"
            ),
            patch.object(OvalTrackScenario, "_read_spawn_pose", return_value=None),
            patch.object(OvalTrackScenario, "initialize") as initialize,
            patch.object(OvalTrackScenario, "setup") as setup,
        ):
            with self.assertRaisesRegex(
                ValueError, "Missing or invalid generated spawn"
            ):
                ScenarioServerNode._read_playground(node, "oval_track_scenario")
            initialize.assert_not_called()
            setup.assert_not_called()

    def test_manual_ego_reset_is_serialized_and_managed_reset_is_delegated(self):
        for managed in (False, True):
            node = SimpleNamespace(
                get_parameter=lambda _: SimpleNamespace(value=managed),
                world_lock=threading.RLock(),
                _reset_ego_locked=Mock(return_value="reset"),
            )
            result = ScenarioServerNode.reset_ego_callback(
                node, None, SimpleNamespace()
            )
            if managed:
                self.assertFalse(result.success)
                self.assertIn("/sim/reset", result.message)
                node._reset_ego_locked.assert_not_called()
            else:
                self.assertEqual(result, "reset")
                node._reset_ego_locked.assert_called_once()

    def test_managed_prepare_rejects_legacy_plugin_before_cleanup_or_setup(self):
        for selector in (
            "oval_track_scenario",
            "carla_scenarios.scenarios.oval_track_scenario",
            "playground/oval_track_scenario",
        ):
            node = SimpleNamespace(
                current_scenario_name="existing",
                world_lock=threading.RLock(),
                _read_playground=Mock(),
                _cleanup_bridge=Mock(),
                get_logger=Mock(),
            )
            response = ScenarioServerNode.prepare_experiment_callback(
                node, SimpleNamespace(scenario_name=selector), SimpleNamespace()
            )
            self.assertFalse(response.success)
            self.assertIn("manual simulation launch", response.message)
            self.assertEqual(response.previous_scenario, "existing")
            node._read_playground.assert_not_called()
            node._cleanup_bridge.assert_not_called()
