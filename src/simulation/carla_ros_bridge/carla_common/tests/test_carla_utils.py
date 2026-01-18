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
"""Tests for CARLA client utilities."""

import sys
from unittest.mock import MagicMock, patch

import pytest


class TestConnectCarla:
    """Tests for connect_carla function."""

    def test_connect_carla_success(self):
        """Test successful connection to CARLA."""
        mock_carla = MagicMock()
        mock_client = MagicMock()
        mock_carla.Client.return_value = mock_client

        with patch.dict(sys.modules, {"carla": mock_carla}):
            # Re-import to pick up the mock
            from carla_common import carla_utils

            # Reload to use mocked carla
            import importlib

            importlib.reload(carla_utils)

            client = carla_utils.connect_carla("localhost", 2000, 10.0)

            mock_carla.Client.assert_called_once_with("localhost", 2000)
            mock_client.set_timeout.assert_called_once_with(10.0)
            assert client == mock_client

    def test_connect_carla_custom_params(self):
        """Test connection with custom parameters."""
        mock_carla = MagicMock()
        mock_client = MagicMock()
        mock_carla.Client.return_value = mock_client

        with patch.dict(sys.modules, {"carla": mock_carla}):
            from carla_common import carla_utils

            import importlib

            importlib.reload(carla_utils)

            client = carla_utils.connect_carla("192.168.1.100", 3000, 30.0)

            mock_carla.Client.assert_called_once_with("192.168.1.100", 3000)
            mock_client.set_timeout.assert_called_once_with(30.0)


class TestFindEgoVehicle:
    """Tests for find_ego_vehicle function."""

    def test_find_ego_vehicle_found(self):
        """Test finding ego vehicle when it exists."""
        mock_carla = MagicMock()

        # Create mock vehicles
        mock_ego = MagicMock()
        mock_ego.attributes = {"role_name": "ego_vehicle"}

        mock_other = MagicMock()
        mock_other.attributes = {"role_name": "npc"}

        mock_actors = MagicMock()
        mock_actors.filter.return_value = [mock_other, mock_ego]

        mock_world = MagicMock()
        mock_world.get_actors.return_value = mock_actors

        with patch.dict(sys.modules, {"carla": mock_carla}):
            from carla_common import carla_utils

            import importlib

            importlib.reload(carla_utils)

            result = carla_utils.find_ego_vehicle(mock_world, "ego_vehicle")

            mock_world.get_actors.assert_called_once()
            mock_actors.filter.assert_called_once_with("vehicle.*")
            assert result == mock_ego

    def test_find_ego_vehicle_not_found(self):
        """Test when ego vehicle does not exist."""
        mock_carla = MagicMock()

        mock_other = MagicMock()
        mock_other.attributes = {"role_name": "npc"}

        mock_actors = MagicMock()
        mock_actors.filter.return_value = [mock_other]

        mock_world = MagicMock()
        mock_world.get_actors.return_value = mock_actors

        with patch.dict(sys.modules, {"carla": mock_carla}):
            from carla_common import carla_utils

            import importlib

            importlib.reload(carla_utils)

            result = carla_utils.find_ego_vehicle(mock_world, "ego_vehicle")

            assert result is None

    def test_find_ego_vehicle_custom_role_name(self):
        """Test finding vehicle with custom role name."""
        mock_carla = MagicMock()

        mock_hero = MagicMock()
        mock_hero.attributes = {"role_name": "hero"}

        mock_actors = MagicMock()
        mock_actors.filter.return_value = [mock_hero]

        mock_world = MagicMock()
        mock_world.get_actors.return_value = mock_actors

        with patch.dict(sys.modules, {"carla": mock_carla}):
            from carla_common import carla_utils

            import importlib

            importlib.reload(carla_utils)

            result = carla_utils.find_ego_vehicle(mock_world, "hero")

            assert result == mock_hero

    def test_find_ego_vehicle_empty_world(self):
        """Test when world has no vehicles."""
        mock_carla = MagicMock()

        mock_actors = MagicMock()
        mock_actors.filter.return_value = []

        mock_world = MagicMock()
        mock_world.get_actors.return_value = mock_actors

        with patch.dict(sys.modules, {"carla": mock_carla}):
            from carla_common import carla_utils

            import importlib

            importlib.reload(carla_utils)

            result = carla_utils.find_ego_vehicle(mock_world, "ego_vehicle")

            assert result is None
