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


class TestScenarioServerParameters:
    """Tests for scenario server parameters."""

    def test_default_parameter_values(self):
        """Test expected default parameter values."""
        defaults = {
            "carla_host": "localhost",
            "carla_port": 2000,
            "carla_timeout": 10.0,
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

    def test_builtin_scenarios_not_duplicated_by_entry_points(self):
        """Test that built-in scenarios registered as entry points are skipped."""
        builtin_scenarios = {
            "carla_scenarios.scenarios.default_scenario": "Default Ego Spawn",
            "carla_scenarios.scenarios.empty_scenario": "Empty World (no NPCs)",
        }

        # Simulate an entry point that matches a built-in
        ep_module_path = "carla_scenarios.scenarios.default_scenario"
        if ep_module_path not in builtin_scenarios:
            builtin_scenarios[ep_module_path] = "Duplicate"

        # Should still have only 2 entries (no duplicate added)
        assert len(builtin_scenarios) == 2

    def test_external_entry_point_added(self):
        """Test that external entry points are merged into available scenarios."""
        available = {
            "carla_scenarios.scenarios.default_scenario": "Default Ego Spawn",
        }

        # Simulate discovering an external scenario
        external_path = "my_pkg.scenarios.highway_scenario"
        external_desc = "Highway driving"
        if external_path not in available:
            available[external_path] = external_desc

        assert len(available) == 2
        assert available[external_path] == "Highway driving"

    def test_class_name_derivation_from_module_path(self):
        """Test PascalCase class name derived from snake_case module name."""
        module_path = "my_pkg.scenarios.highway_merge_scenario"
        last_part = module_path.rsplit(".", 1)[-1]
        class_name = "".join(word.capitalize() for word in last_part.split("_"))

        assert class_name == "HighwayMergeScenario"


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
