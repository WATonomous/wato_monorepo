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
"""Manual/scored session boundaries without starting ROS nodes."""

import json
import unittest
from pathlib import Path
from types import FunctionType, MethodType, SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

from simulation_runtime.assets import catalog_directory
from carla_scenarios.definitions import load_definition
from simulation_manager.simulation_runner import (
    ReadinessDeadlineExpired,
    SimulationRunner,
)

CONFIG = catalog_directory()


def runner(kind="playground", state="IDLE"):
    """Bind real runner methods to inert state without creating a ROS node."""
    node = SimpleNamespace(
        kind=kind,
        experiment="controller_snake",
        state=state,
        server_active=True,
        definitions=CONFIG,
        samples={},
        stack=Mock(prepared=False),
        tf_buffer=MagicMock(),
        path_pub=MagicMock(),
        speed_pub=MagicMock(),
        get_logger=MagicMock(),
        _world_endpoints_ready=Mock(return_value=True),
        get_parameter=lambda name: SimpleNamespace(
            value={"session_id": "test", "results_directory": "/results"}.get(name)
        ),
    )
    for name, method in SimulationRunner.__dict__.items():
        if isinstance(method, FunctionType) and name != "__init__":
            setattr(node, name, MethodType(method, node))
    node._world_endpoints_ready = Mock(return_value=True)
    node._prepare_world = Mock()
    node._pause_world = Mock()
    node._call = Mock()
    node._write_configuration = Mock()
    node._bridge_readiness = Mock(return_value=None)
    return node


class TestManualRunner(unittest.TestCase):
    def test_open_does_not_create_a_scored_attempt(self):
        node = runner()
        node._new_attempt = Mock(side_effect=AssertionError("manual attempt"))
        with patch.object(
            Path, "mkdir", side_effect=AssertionError("manual result directory")
        ):
            response = node._open(
                SimpleNamespace(scenario_name="playground/empty"), SimpleNamespace()
            )
        self.assertTrue(response.success, response.message)
        self.assertIsNone(json.loads(response.message)["result_directory"])
        self.assertEqual(node.run_id, "")
        node._new_attempt.assert_not_called()
        node._write_configuration.assert_not_called()

    def test_ready_leaves_manual_world_running_without_play(self):
        node = runner()
        node._clear_attempt_state()
        node.definition = load_definition(CONFIG, "playgrounds", "empty")
        node._readiness = Mock(return_value=None)
        node._do_ready()
        self.assertEqual(node.state, "RUNNING")
        self.assertFalse(node.accepting_samples)
        self.assertEqual(node.events, [])
        self.assertEqual(node._pause_world.call_args.args, (False,))

    def test_pause_resume_reset_close_do_not_evaluate_or_start_application_nodes(self):
        node = runner(state="RUNNING")
        node._clear_attempt_state()
        node.state = "RUNNING"
        node.definition = {}
        node._node_state = Mock(side_effect=AssertionError("application lifecycle"))
        node._new_attempt = Mock(side_effect=AssertionError("manual attempt"))
        node._do_ready = Mock()
        with patch.object(
            Path, "mkdir", side_effect=AssertionError("manual result directory")
        ):
            node._do_pause()
            node._do_resume()
            node._do_reset()
            node._finish("interrupted", ["manual close"])
            node._do_close()
        self.assertEqual(node.state, "IDLE")
        self.assertEqual(node.events, [])
        self.assertIsNone(node.outcome)
        node._new_attempt.assert_not_called()
        node._node_state.assert_not_called()

    def test_scenario_open_still_allocates_results(self):
        node = runner()
        with patch.object(Path, "mkdir") as mkdir:
            response = node._open(
                SimpleNamespace(scenario_name="scenario/controller_snake"),
                SimpleNamespace(),
            )
        self.assertTrue(response.success, response.message)
        mkdir.assert_called_once_with(parents=True, exist_ok=False)
        self.assertTrue(node.run_id)
        self.assertIsNotNone(node.result_directory)
        node._write_configuration.assert_called_once()

    def test_scenario_ready_still_pauses_before_play(self):
        node = runner(kind="scenario")
        node._clear_attempt_state()
        node.definition = load_definition(CONFIG, "scenarios", "controller_snake")
        node._readiness = Mock(return_value=None)
        node._do_ready()
        self.assertEqual(node.state, "READY")
        self.assertEqual(node._pause_world.call_args.args, (True,))
        self.assertEqual(node.events[0]["event"], "ready")

    def test_scenario_pause_resume_keeps_execution_accounting(self):
        node = runner(kind="scenario")
        node._clear_attempt_state()
        node.state = "RUNNING"

        node._do_pause()
        node._do_resume()

        self.assertEqual(node.state, "RUNNING")
        self.assertEqual([event["event"] for event in node.events], ["pause", "resume"])
        self.assertGreaterEqual(node.paused_wall_s, 0.0)
        self.assertEqual(
            [call.args[0] for call in node._pause_world.call_args_list], [True, False]
        )

    def test_scenario_reset_restarts_stack_and_creates_a_fresh_attempt(self):
        node = runner(kind="scenario")
        node._clear_attempt_state()
        node.state, node.outcome, node.run_id = "FINISHED", "assertion_failure", "old"
        node.definition = load_definition(CONFIG, "scenarios", "controller_snake")
        node._do_ready = Mock()
        with patch.object(Path, "mkdir") as mkdir:
            node._do_reset()

        mkdir.assert_called_once_with(parents=True, exist_ok=False)
        self.assertNotEqual(node.run_id, "old")
        self.assertIsNone(node.outcome)
        node.stack.stop.assert_called_once()
        node._do_ready.assert_called_once()

    def test_readiness_cannot_become_ready_after_its_deadline(self):
        node = runner(kind="scenario")
        node._clear_attempt_state()
        node.definition = load_definition(CONFIG, "scenarios", "controller_snake")
        node.definition["deadlines"]["readiness_s"] = 1.0
        clock = [0.0]

        def delayed_readiness(**kwargs):
            clock[0] = 2.0
            return None

        node._readiness = Mock(side_effect=delayed_readiness)
        with patch(
            "simulation_manager.simulation_runner.time.monotonic",
            side_effect=lambda: clock[0],
        ):
            with self.assertRaisesRegex(RuntimeError, "Readiness timed out"):
                node._do_ready()
        self.assertEqual(node.state, "PREPARING")
        node._pause_world.assert_not_called()

    def test_expiring_bridge_probe_preserves_previous_readiness_failure(self):
        node = runner(kind="scenario")
        node._clear_attempt_state()
        node.definition = load_definition(CONFIG, "scenarios", "controller_snake")
        node.definition["deadlines"]["readiness_s"] = 1.0
        node.stack.prepared = True
        reason = "ego moved away from its initial spawn during preparation"
        node._readiness = Mock(return_value=reason)
        clock = [0.0]

        def bridge_probe(**kwargs):
            if node._readiness.called:
                clock[0] = 1.1
                raise ReadinessDeadlineExpired("Bridge readiness deadline expired")
            return None

        node._bridge_readiness = Mock(side_effect=bridge_probe)
        with (
            patch(
                "simulation_manager.simulation_runner.time.monotonic",
                side_effect=lambda: clock[0],
            ),
            patch("simulation_manager.simulation_runner.time.sleep"),
        ):
            with self.assertRaisesRegex(RuntimeError, reason) as caught:
                node._do_ready()

        self.assertNotIn("Bridge readiness deadline expired", str(caught.exception))
        node._pause_world.assert_not_called()
        self.assertEqual(node.state, "PREPARING")

    def test_bridge_readiness_propagates_budget_expiry(self):
        node = runner(kind="scenario")
        node.sim_time = 1.0
        node.last_clock_wall = 0.0
        node._fresh = Mock(return_value=True)
        node._node_state = Mock(side_effect=ReadinessDeadlineExpired("expired"))
        with patch(
            "simulation_manager.simulation_runner.time.monotonic", return_value=0.0
        ):
            with self.assertRaises(ReadinessDeadlineExpired):
                SimulationRunner._bridge_readiness(node, deadline=1.0)
