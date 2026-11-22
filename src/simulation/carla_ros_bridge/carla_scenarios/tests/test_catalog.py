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
"""Catalog compatibility and the manual/scored execution boundary."""

import json
import threading
import unittest
from pathlib import Path
from types import FunctionType, MethodType, SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import carla
from carla_scenarios.definitions import (
    LEGACY_PLAYGROUNDS,
    load_definition,
    world_definition,
)
from carla_scenarios.scenario_server_node import ScenarioServerNode
from carla_scenarios.simulation_runner import SimulationRunner
from carla_scenarios.worlds.configured_world import ConfiguredWorld
from lifecycle_msgs.msg import State, Transition

CONFIG = Path(__file__).resolve().parents[1] / "config"


class TestCatalog(unittest.TestCase):
    def test_invalid_inputs_report_a_definition_error(self):
        definition = {
            "version": 1,
            "id": "controller_snake",
            "stack": "controller_pure_pursuit",
            "completion": "maneuver_finished",
            "inputs": [],
        }
        with patch(
            "carla_scenarios.definitions.yaml.safe_load", return_value=definition
        ):
            with self.assertRaisesRegex(ValueError, "inputs must be a mapping"):
                load_definition(CONFIG, "scenarios", "controller_snake")

    def test_town_variants_preserve_legacy_settings(self):
        for name, vehicles, walkers, index, weather, crossing in (
            ("town10hd_default", 20, 40, 0, "ClearNoon", 0.0),
            ("town10hd_light", 5, 10, 5, "ClearSunset", 0.0),
            ("town10hd_heavy", 50, 100, 25, "CloudyNoon", 0.2),
            ("empty", 0, 0, 10, "ClearNoon", 0.0),
            ("custom", 10, 20, 10, "ClearNoon", 0.1),
        ):
            with self.subTest(name=name):
                definition = load_definition(CONFIG, "playgrounds", name)
                self.assertEqual(definition["map"], {"town": "Town10HD_Opt"})
                self.assertEqual(definition["spawn"]["index"], index)
                self.assertEqual(definition["weather"], weather)
                self.assertEqual(definition["traffic"]["vehicles"], vehicles)
                self.assertEqual(definition["traffic"]["pedestrians"], walkers)
                self.assertEqual(
                    definition["traffic"]["pedestrian_crossing_factor"], crossing
                )

    def test_legacy_selectors_resolve_to_one_implementation(self):
        for legacy, name in LEGACY_PLAYGROUNDS.items():
            with self.subTest(legacy=legacy):
                expected = load_definition(CONFIG, "playgrounds", name)
                self.assertEqual(
                    world_definition(CONFIG, "playground", legacy), expected
                )
                self.assertEqual(
                    world_definition(CONFIG, "playground", legacy.rsplit(".", 1)[1]),
                    expected,
                )

    def test_fixtures_belong_to_scenarios_and_do_not_mutate_base_world(self):
        original = load_definition(CONFIG, "playgrounds", "oval")
        for name, distance in (
            ("controller_snake", 82.0),
            ("parked_car_straight", 80.0),
        ):
            definition = load_definition(CONFIG, "scenarios", name)
            resolved = world_definition(CONFIG, "scenario", name)
            self.assertEqual(definition["playground"], "oval")
            self.assertTrue(resolved["spawn"]["straight_start"])
            self.assertEqual(resolved["required_straight_m"], distance)
            self.assertEqual(resolved["goal_distance_m"], 80.0)
        self.assertEqual(load_definition(CONFIG, "playgrounds", "oval"), original)
        self.assertNotIn("parked", original)
        self.assertNotIn("straight_start", original["spawn"])
        for name in ("snake_straight", "parked_car_straight"):
            self.assertFalse((CONFIG / "playgrounds" / f"{name}.yaml").is_file())


def runner(kind="playground", state="IDLE"):
    """Bind real runner methods to inert state without creating a ROS node."""
    node = SimpleNamespace(
        kind=kind,
        state=state,
        server_active=True,
        definitions=CONFIG,
        samples={},
        tf_buffer=MagicMock(),
        path_pub=MagicMock(),
        speed_pub=MagicMock(),
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
        node._pause_world.assert_not_called()

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
        node._pause_world.assert_called_once_with(True)
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
        nodes = ["/action/fake_planner/fake_planner_node", "/action/pure_pursuit_node"]
        states = dict.fromkeys(nodes, State.PRIMARY_STATE_ACTIVE)
        node._node_state = Mock(side_effect=lambda name: states[name])

        def transition(name, change):
            states[name] = {
                Transition.TRANSITION_DEACTIVATE: State.PRIMARY_STATE_INACTIVE,
                Transition.TRANSITION_CLEANUP: State.PRIMARY_STATE_UNCONFIGURED,
                Transition.TRANSITION_CONFIGURE: State.PRIMARY_STATE_INACTIVE,
                Transition.TRANSITION_ACTIVATE: State.PRIMARY_STATE_ACTIVE,
            }[change]

        node._transition = Mock(side_effect=transition)
        node._do_ready = Mock()
        with patch.object(Path, "mkdir") as mkdir:
            node._do_reset()

        mkdir.assert_called_once_with(parents=True, exist_ok=False)
        self.assertNotEqual(node.run_id, "old")
        self.assertIsNone(node.outcome)
        self.assertTrue(
            all(state == State.PRIMARY_STATE_ACTIVE for state in states.values())
        )
        self.assertEqual(node._transition.call_count, 8)
        node._do_ready.assert_called_once()


class TestConfiguredWorld(unittest.TestCase):
    def test_custom_world_preserves_spawn_traffic_and_prop_reservations(self):
        definition = load_definition(CONFIG, "playgrounds", "custom")
        world = ConfiguredWorld(definition)
        world.world = MagicMock()
        points = [carla.Transform(carla.Location(x=float(i))) for i in range(30)]
        world.world.get_map.return_value.get_spawn_points.return_value = points
        world.world.get_blueprint_library.return_value.filter.return_value = [object()]
        world._load_map = Mock()
        world.spawn_ego_vehicle = Mock(return_value=MagicMock())
        world.spawn_vehicles = Mock()
        world.spawn_pedestrians = Mock()
        world._actor_geometry = Mock(return_value={})

        self.assertTrue(world.setup())

        world._load_map.assert_called_once_with("Town10HD_Opt")
        self.assertIs(world.spawn_ego_vehicle.call_args.args[0], points[10])
        self.assertEqual(world.world.try_spawn_actor.call_count, 8)
        remaining = [point for point in points if point is not points[10]][8:]
        world.spawn_vehicles.assert_called_once_with(10, remaining)
        world.spawn_pedestrians.assert_called_once_with(20, 0.1)
        self.assertFalse(world.held)
        self.assertEqual(len(world.owned_actors), 8)

    def test_opendrive_worlds_share_legacy_spawn_fallback(self):
        for name in ("oval", "ring_road"):
            with self.subTest(name=name):
                definition = load_definition(CONFIG, "playgrounds", name)
                world = ConfiguredWorld(definition, Path(name + ".xodr"))
                world.world = MagicMock()
                world.world.get_map.return_value.get_spawn_points.return_value = []
                transform = carla.Transform(carla.Location(z=0.0))
                waypoint = SimpleNamespace(
                    transform=transform, road_id=1, lane_id=-1, s=0.0
                )
                world.world.get_map.return_value.generate_waypoints.return_value = [
                    waypoint
                ]
                world._load_xodr_map = Mock()
                world.spawn_ego_vehicle = Mock(return_value=MagicMock())
                world._actor_geometry = Mock(return_value={})
                with patch.object(Path, "read_bytes", return_value=b"map"):
                    self.assertTrue(world.setup())
                world._load_xodr_map.assert_called_once_with(name + ".xodr")
                self.assertAlmostEqual(transform.location.z, 0.5)
                self.assertIs(world.spawn_ego_vehicle.call_args.args[0], transform)

    def test_cleanup_only_releases_owned_actors(self):
        world = ConfiguredWorld(load_definition(CONFIG, "playgrounds", "empty"))
        world.world = MagicMock()
        owned = MagicMock(type_id="vehicle.mini.cooper", is_alive=True)
        owned.destroy.return_value = True
        world.owned_actors = [owned]
        world.cleanup()
        owned.destroy.assert_called_once()
        world.world.get_actors.assert_not_called()
        self.assertEqual(world.owned_actors, [])


class TestServerPreparation(unittest.TestCase):
    def prepare(self, kind, *, initialized=True):
        configured = MagicMock()
        configured.initialize.return_value = initialized
        configured.definition = {"seed": 42}
        configured.geometry = {"map": {}}
        configured.provenance = None
        configured.owned_actors = []
        world = MagicMock()
        world.get_settings.return_value = SimpleNamespace(
            synchronous_mode=True,
            fixed_delta_seconds=1.0 / 60.0,
            substepping=True,
            max_substep_delta_time=0.002,
            max_substeps=10,
        )
        world.get_snapshot.return_value.timestamp.elapsed_seconds = 1.0
        sensor = world.spawn_actor.return_value
        sensor.id = 42
        node = SimpleNamespace(
            current_scenario_name="",
            world_lock=threading.RLock(),
            _read_playground=Mock(return_value=configured),
            _cleanup_bridge=Mock(),
            _publish_control_enabled=Mock(),
            _unload_scenario=Mock(),
            _publish_scenario_status=Mock(),
            _advance_world=Mock(),
            _apply_simulation_settings=Mock(),
            collision_publisher=Mock(),
            collision_sensor=None,
            get_logger=MagicMock(),
            carla_client=SimpleNamespace(
                get_world=lambda: world,
                get_server_version=lambda: "0.10.0",
                get_client_version=lambda: "0.10.0",
            ),
        )
        response = ScenarioServerNode.prepare_experiment_callback(
            node, SimpleNamespace(scenario_name=f"{kind}/oval"), SimpleNamespace()
        )
        self.assertEqual(response.success, initialized, response.message)
        node.response = response
        return node, world, configured

    def test_failed_initialization_does_not_spawn_world_content(self):
        node, world, configured = self.prepare("playground", initialized=False)

        self.assertIn("World initialization failed", node.response.message)
        configured.setup.assert_not_called()
        world.spawn_actor.assert_not_called()

    def test_manual_prepare_unpauses_without_collision_evaluator(self):
        node, world, configured = self.prepare("playground")

        world.spawn_actor.assert_not_called()
        self.assertEqual(configured.owned_actors, [])
        node.collision_publisher.publish.assert_not_called()
        self.assertFalse(node.paused)
        self.assertEqual(node._publish_control_enabled.call_args.args, (True,))

    def test_scored_prepare_keeps_collision_observation_and_command_fence(self):
        node, world, configured = self.prepare("scenario")

        world.spawn_actor.assert_called_once()
        self.assertEqual(configured.owned_actors, [node.collision_sensor])
        node.collision_sensor.listen.assert_called_once()
        node.collision_publisher.publish.assert_called_once()
        self.assertEqual(node._publish_control_enabled.call_args.args, (False,))
