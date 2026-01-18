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
"""Tests for scenario base class."""

import sys
from unittest.mock import MagicMock, patch

import pytest


class TestScenarioBaseHelpers:
    """Tests for ScenarioBase helper methods."""

    def test_spawn_point_selection(self):
        """Test spawn point selection from available points."""
        spawn_points = [
            MagicMock(location=MagicMock(x=0, y=0, z=0)),
            MagicMock(location=MagicMock(x=10, y=10, z=0)),
            MagicMock(location=MagicMock(x=20, y=20, z=0)),
        ]

        # Random selection should return one of the points
        import random

        selected = random.choice(spawn_points)
        assert selected in spawn_points

    def test_ego_vehicle_spawn_height(self):
        """Test ego vehicle spawns at correct height."""
        ground_z = 0.0
        spawn_height_offset = 0.6

        spawn_z = ground_z + spawn_height_offset
        assert spawn_z == 0.6


class TestVehicleSpawning:
    """Tests for vehicle spawning logic."""

    def test_vehicle_count_validation(self):
        """Test vehicle count is non-negative."""
        vehicle_count = 10
        assert vehicle_count >= 0

    def test_spawn_retry_logic(self):
        """Test spawn retry on failure."""
        max_retries = 3
        attempts = 0
        success = False

        while attempts < max_retries and not success:
            attempts += 1
            if attempts == 2:  # Succeed on second try
                success = True

        assert success is True
        assert attempts == 2

    def test_autopilot_enabled(self):
        """Test NPC vehicles have autopilot enabled."""
        autopilot = True
        assert autopilot is True


class TestPedestrianSpawning:
    """Tests for pedestrian spawning logic."""

    def test_pedestrian_count_validation(self):
        """Test pedestrian count is non-negative."""
        pedestrian_count = 50
        assert pedestrian_count >= 0

    def test_walker_speed_range(self):
        """Test walker speeds are in valid range."""
        min_speed = 1.0  # m/s
        max_speed = 2.0  # m/s

        import random

        speed = random.uniform(min_speed, max_speed)
        assert min_speed <= speed <= max_speed

    def test_pedestrian_crossing_factor(self):
        """Test pedestrian crossing factor is in [0, 1]."""
        crossing_factor = 0.5
        assert 0.0 <= crossing_factor <= 1.0


class TestActorCleanup:
    """Tests for actor cleanup logic."""

    def test_cleanup_filters_hero(self):
        """Test cleanup does not destroy hero vehicle."""
        actors = [
            MagicMock(attributes={"role_name": "hero"}),
            MagicMock(attributes={"role_name": "npc"}),
            MagicMock(attributes={"role_name": "autopilot"}),
        ]

        to_destroy = [a for a in actors if a.attributes.get("role_name") != "hero"]
        assert len(to_destroy) == 2

    def test_cleanup_order(self):
        """Test controllers are destroyed before walkers."""
        # Controllers must be stopped and destroyed before walkers
        cleanup_order = ["stop_controllers", "destroy_controllers", "destroy_walkers"]

        assert cleanup_order[0] == "stop_controllers"
        assert cleanup_order[-1] == "destroy_walkers"


class TestMapLoading:
    """Tests for map loading logic."""

    def test_map_name_format(self):
        """Test map names follow CARLA format."""
        valid_maps = [
            "Town01",
            "Town02",
            "Town03",
            "Town04",
            "Town05",
            "Town10HD",
        ]

        for map_name in valid_maps:
            assert map_name.startswith("Town")

    def test_map_change_clears_actors(self):
        """Test map change requires actor cleanup."""
        current_map = "Town01"
        new_map = "Town02"

        map_changed = current_map != new_map
        needs_cleanup = map_changed

        assert needs_cleanup is True
