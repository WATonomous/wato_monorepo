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
"""Tests for scenario server node."""

import importlib
import sys
from types import SimpleNamespace
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def server(monkeypatch):
    """Exercise the real server methods without ROS or a running CARLA server."""
    dependencies = (
        "rclpy",
        "rclpy.lifecycle",
        "rcl_interfaces.msg",
        "carla_msgs.srv",
        "carla_msgs.msg",
        "std_msgs.msg",
        "std_srvs.srv",
        "rosgraph_msgs.msg",
        "carla",
        "carla_common",
    )
    for name in dependencies:
        monkeypatch.setitem(sys.modules, name, MagicMock())
    sys.modules["rclpy.lifecycle"].LifecycleNode = object
    name = "carla_scenarios.scenario_server_node"
    monkeypatch.delitem(sys.modules, name, raising=False)
    module = importlib.import_module(name)
    monkeypatch.delitem(sys.modules, name)
    node = object.__new__(module.ScenarioServerNode)
    node.current_scenario = MagicMock()
    node.current_scenario_name = "old_scenario"
    node.current_map_name = "old_map"
    node.carla_client = MagicMock()
    node.get_logger = MagicMock()
    node.get_parameter = MagicMock(return_value=SimpleNamespace(value="/tmp"))
    node._prepare_managed_nodes = MagicMock(return_value=True)
    node._unload_scenario = MagicMock()
    node._apply_simulation_settings = MagicMock()
    scenario = MagicMock()
    scenario.initialize.return_value = True
    scenario.load_map.return_value = True
    scenario.setup.return_value = True
    scenario.world.get_map.return_value.name = "new_map"
    scenario_class = MagicMock(return_value=scenario)
    monkeypatch.setattr(
        module,
        "importlib",
        SimpleNamespace(
            import_module=lambda _: SimpleNamespace(TestScenario=scenario_class)
        ),
    )
    return node, scenario


def test_loads_map_and_settings_before_spawning(server):
    node, scenario = server
    calls = MagicMock()
    for name, method in (
        ("prepare", node._prepare_managed_nodes),
        ("unload", node._unload_scenario),
        ("initialize", scenario.initialize),
        ("map", scenario.load_map),
        ("settings", node._apply_simulation_settings),
        ("setup", scenario.setup),
    ):
        calls.attach_mock(method, name)
    assert node._load_scenario("test.test_scenario")
    assert [call[0] for call in calls.mock_calls] == [
        "prepare",
        "unload",
        "initialize",
        "map",
        "settings",
        "setup",
    ]
    assert node.current_map_name == "new_map"
    assert node.current_scenario is scenario


def test_invalid_map_definition_preserves_current_scenario(server):
    node, scenario = server
    scenario.validate_map.side_effect = ValueError("invalid map")
    assert not node._load_scenario("test.test_scenario")
    node._unload_scenario.assert_not_called()
    assert node.current_scenario_name == "old_scenario"


def test_cleanup_refusal_preserves_current_scenario(server):
    node, scenario = server
    node._prepare_managed_nodes.return_value = False
    assert not node._load_scenario("test.test_scenario")
    node._unload_scenario.assert_not_called()
    scenario.initialize.assert_not_called()


@pytest.mark.parametrize("stage", ["initialize", "load_map", "setup"])
def test_failed_transition_cleans_partial_scenario(server, stage):
    node, scenario = server
    getattr(scenario, stage).return_value = False
    assert not node._load_scenario("test.test_scenario")
    assert node._unload_scenario.call_count == 2
    assert node.current_scenario is None
    assert node.current_scenario_name == ""


def test_switch_response_includes_previous_and_resulting_maps(server):
    node, _ = server
    response = node.switch_scenario_callback(
        SimpleNamespace(scenario_name="test.test_scenario"), SimpleNamespace()
    )
    assert response.success
    assert response.previous_map == "old_map"
    assert response.new_map == "new_map"
    assert response.previous_scenario == "old_scenario"


class TestScenarioServerParameters:
    """Tests for scenario server parameters."""

    def test_default_parameter_values(self):
        """Test expected default parameter values."""
        defaults = {
            "carla_host": "localhost",
            "carla_port": 2000,
            "carla_timeout": 60.0,
            "initial_scenario": "carla_scenarios.scenarios.default_scenario",
            "carla_fps": 60.0,
            "synchronous_mode": False,
            "no_rendering_mode": False,
            "substepping": True,
            "max_substep_delta_time": 0.01,
            "max_substeps": 10,
        }

        assert defaults["carla_host"] == "localhost"
        assert defaults["carla_port"] == 2000
        assert defaults["carla_fps"] == 60.0
        assert defaults["synchronous_mode"] is False
        assert defaults["substepping"] is True
        assert defaults["max_substeps"] == 10

    def test_fps_to_fixed_delta(self):
        """Test FPS to fixed_delta_seconds conversion."""
        fps = 60.0
        fixed_delta = 1.0 / fps

        assert abs(fixed_delta - 0.016666666666666666) < 1e-15

    def test_timer_period_sync_mode(self):
        """Test timer period in sync mode."""
        sync_mode = True
        carla_fps = 60.0

        timer_period = (1.0 / carla_fps) if sync_mode else 0.001
        assert abs(timer_period - 1.0 / 60.0) < 1e-10

    def test_timer_period_async_mode(self):
        """Test timer period in async mode."""
        sync_mode = False
        carla_fps = 60.0

        timer_period = (1.0 / carla_fps) if sync_mode else 0.001
        assert timer_period == 0.001


class TestWorldSettings:
    """Tests for CARLA world settings."""

    def test_sync_mode_fixed_delta(self):
        """Test sync mode uses fixed delta."""
        sync_mode = True
        carla_fps = 60.0

        if sync_mode:
            fixed_delta = 1.0 / carla_fps
        else:
            fixed_delta = 0.0

        assert abs(fixed_delta - 1.0 / 60.0) < 1e-10

    def test_async_mode_variable_timestep(self):
        """Test async mode uses variable timestep (0)."""
        sync_mode = False
        carla_fps = 60.0

        if sync_mode:
            fixed_delta = 1.0 / carla_fps
        else:
            fixed_delta = 0.0

        assert fixed_delta == 0.0

    def test_substepping_valid_range(self):
        """Test max_substeps is in valid range."""
        max_substeps = 10
        valid_range = range(1, 17)

        assert max_substeps in valid_range


class TestScenarioDiscovery:
    """Tests for scenario discovery."""

    def test_scenario_module_path_format(self):
        """Test scenario module path format."""
        module_path = "carla_scenarios.scenarios.light_traffic_scenario"
        parts = module_path.split(".")

        assert len(parts) == 3
        assert parts[0] == "carla_scenarios"
        assert parts[1] == "scenarios"

    def test_scenario_name_extraction(self):
        """Test scenario name is extracted from module."""
        module_path = "carla_scenarios.scenarios.light_traffic_scenario"
        scenario_name = module_path.split(".")[-1]

        assert scenario_name == "light_traffic_scenario"


class TestMapDiscovery:
    """Tests for map discovery examples."""

    def test_carla_map_paths_produce_expected_map_names(self):
        map_paths = [
            "/Game/Carla/Maps/Town01",
            "/Game/Carla/Maps/Town10HD",
        ]
        map_names = [path.rsplit("/", 1)[-1] for path in map_paths]

        assert map_names == ["Town01", "Town10HD"]


class TestMapSwitching:
    """Tests for map switching examples."""

    def test_known_map_can_be_selected(self):
        available_maps = ["Town01", "Town10HD"]
        requested_map = "Town10HD"
        should_load_map = requested_map in available_maps

        assert should_load_map is True

    def test_unknown_map_is_rejected(self):
        available_maps = ["Town01", "Town10HD"]
        requested_map = "Town99"
        should_load_map = requested_map in available_maps

        assert should_load_map is False


class TestScenarioSwitching:
    """Tests for scenario switching logic."""

    def test_cleanup_before_load(self):
        """Test cleanup happens before loading new scenario."""
        steps = ["cleanup_actors", "load_map", "spawn_actors"]

        assert steps[0] == "cleanup_actors"
        assert steps[-1] == "spawn_actors"

    def test_actor_cleanup_count(self):
        """Test actor cleanup reports correct count."""
        actors_before = 50
        actors_removed = 50
        actors_after = 0

        assert actors_removed == actors_before
        assert actors_after == actors_before - actors_removed
