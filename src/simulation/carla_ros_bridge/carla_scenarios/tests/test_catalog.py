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
"""Validate the entire data catalog and existing world/session boundaries."""

import threading
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import MagicMock, Mock, patch

import carla
from simulation_runtime.assets import load_maneuver, maneuver_directory
from simulation_runtime.sim_presets import launch_command, launch_groups
from carla_scenarios.definitions import (
    load_definition,
    world_definition,
)
from carla_scenarios.scenario_server_node import ScenarioServerNode
from carla_scenarios.worlds.configured_world import ConfiguredWorld

CONFIG = Path(__file__).resolve().parents[1] / "config"


class TestCatalog(unittest.TestCase):
    def test_all_playgrounds_validate(self):
        paths = sorted((CONFIG / "playgrounds").glob("*.yaml"))
        self.assertTrue(paths, "No playground definitions found")
        for path in paths:
            with self.subTest(path=path.name):
                load_definition(CONFIG, "playgrounds", path.stem)

    def test_all_scenarios_and_referenced_assets_validate(self):
        paths = sorted((CONFIG / "scenarios").glob("*.yaml"))
        self.assertTrue(paths, "No scenario definitions found")
        for path in paths:
            with self.subTest(path=path.name):
                definition = load_definition(CONFIG, "scenarios", path.stem)
                world_definition(CONFIG, "scenario", path.stem)
                configuration = {
                    "preset": definition["stack"],
                    "attempt_id": "catalog_validation",
                    "inputs": definition["inputs"],
                }
                for role in launch_groups(definition["stack"], definition["inputs"]):
                    launch_command(role, configuration)

    def test_all_maneuver_files_validate(self):
        directory = maneuver_directory()
        paths = sorted(directory.glob("*.json"))
        self.assertTrue(paths, "No maneuver definitions found")
        for path in paths:
            with self.subTest(path=path.name):
                load_maneuver(path.stem, directory)

    def test_unsupported_scenario_version_is_rejected(self):
        with patch(
            "carla_scenarios.definitions.yaml.safe_load",
            return_value={"version": 1, "id": "controller_snake"},
        ):
            with self.assertRaisesRegex(ValueError, "version 2"):
                load_definition(CONFIG, "scenarios", "controller_snake")


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
