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
"""Exercise startup, ownership and failure cleanup through the adapter interface."""

import json
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from simulation_manager.stack_adapter import SimulationStackAdapter


class TestStackAdapter(unittest.TestCase):
    def make_adapter(self, failure=None, session="owner"):
        calls = []

        def call(service, endpoint, request=None, **kwargs):
            calls.append(endpoint)
            if endpoint == failure:
                raise RuntimeError("injected failure")
            if endpoint.endswith("/status"):
                role = endpoint.split("/")[-2]
                return SimpleNamespace(
                    message=json.dumps(
                        {"session_id": session, "role": role, "process": "stopped"}
                    )
                )
            return SimpleNamespace(
                success=True, result=SimpleNamespace(successful=True)
            )

        return SimulationStackAdapter(call, "owner"), calls

    def definition(self):
        return {
            "stack": "autonomy",
            "inputs": {"map": "oval", "stop_distance_m": 2.0, "max_speed_mps": 3.0},
        }

    def test_startup_uses_one_lifecycle_owner_per_group(self):
        adapter, calls = self.make_adapter()
        adapter.prepare(self.definition(), "attempt_1")
        ordered = [
            endpoint for endpoint in calls if endpoint.endswith(("/start", "/startup"))
        ]
        self.assertEqual(
            ordered,
            [
                "/sim/runtime/world/start",
                "/world_modeling_lifecycle_manager/startup",
                "/sim/runtime/action/start",
                "/action_lifecycle_manager/startup",
            ],
        )
        self.assertTrue(adapter.prepared)
        self.assertEqual(adapter.command_topic, "/sim/attempts/attempt_1/command")
        adapter.stop()
        self.assertEqual(
            [e for e in calls if e.endswith("/stop")],
            ["/sim/runtime/action/stop", "/sim/runtime/world/stop"],
        )
        self.assertFalse(adapter.prepared)

    def test_partial_startup_stops_both_owned_groups(self):
        adapter, calls = self.make_adapter(failure="/action_lifecycle_manager/startup")
        with self.assertRaisesRegex(RuntimeError, "injected failure"):
            adapter.prepare(self.definition(), "attempt_1")
        self.assertEqual(
            [e for e in calls if e.endswith("/stop")],
            ["/sim/runtime/action/stop", "/sim/runtime/world/stop"],
        )
        self.assertEqual(adapter.roles, [])
        self.assertFalse(adapter.prepared)

    def test_wrong_session_cannot_start_or_stop_applications(self):
        adapter, calls = self.make_adapter(session="other")
        with self.assertRaisesRegex(RuntimeError, "ownership mismatch"):
            adapter.prepare(self.definition(), "attempt_1")
        self.assertFalse(
            any(e.endswith(("/start", "/stop", "/startup")) for e in calls)
        )

    def test_failed_stop_retains_group_for_retry(self):
        adapter, calls = self.make_adapter(failure="/sim/runtime/action/stop")
        adapter.roles, adapter.preset, adapter.prepared = (
            ["world", "action"],
            "autonomy",
            True,
        )
        with self.assertRaisesRegex(RuntimeError, "injected failure"):
            adapter.stop()
        self.assertEqual(adapter.roles, ["action"])
        adapter.call = Mock(
            return_value=SimpleNamespace(
                message=json.dumps(
                    {"session_id": "owner", "role": "action", "process": "running"}
                )
            )
        )
        adapter.stop()
        self.assertEqual(adapter.roles, [])

    def test_expired_readiness_does_not_start_applications(self):
        adapter, calls = self.make_adapter()
        with patch(
            "simulation_manager.stack_adapter.time.monotonic", side_effect=[0, 2]
        ):
            with self.assertRaisesRegex(RuntimeError, "deadline expired"):
                adapter.prepare(self.definition(), "attempt_1", timeout_s=1)
        self.assertEqual(calls, [])
