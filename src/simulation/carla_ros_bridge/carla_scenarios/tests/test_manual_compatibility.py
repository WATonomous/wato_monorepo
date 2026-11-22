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
"""Preserved manual reset behavior without ROS or CARLA processes."""

import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock

from carla_scenarios.scenario_server_node import ScenarioServerNode


class TestManualCompatibility(unittest.TestCase):
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
