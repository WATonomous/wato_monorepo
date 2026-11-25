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
"""Map reuse preserves manual town worlds and replaces OpenDRIVE worlds."""

import unittest
from unittest.mock import MagicMock

from carla_scenarios.worlds.configured_world import ConfiguredWorld


class TestMapLoading(unittest.TestCase):
    def make_world(self, current_map):
        world = ConfiguredWorld({"id": "empty", "map": {"town": "Town10HD_Opt"}})
        world.client = MagicMock()
        world.world = MagicMock()
        world.world.get_map.return_value.name = current_map
        return world

    def test_return_from_opendrive_loads_packaged_town(self):
        for name in ("Town10HD_Opt", "/Game/Carla/Maps/Town10HD_Opt"):
            with self.subTest(name=name):
                world = self.make_world("Carla/Maps/OpenDriveMap")

                self.assertTrue(world._load_map(name, wait_time=0))

                world.client.load_world.assert_called_once_with(name)
                self.assertIs(world.world, world.client.get_world.return_value)

    def test_town_switch_reuses_current_world(self):
        for name in ("Town10HD_Opt", "/Game/Carla/Maps/Town10HD_Opt"):
            with self.subTest(name=name):
                world = self.make_world("Carla/Maps/Town10HD_Opt")

                self.assertTrue(world._load_map(name, wait_time=0))

                world.client.load_world.assert_not_called()

    def test_similar_map_name_does_not_skip_reload(self):
        world = self.make_world("Carla/Maps/Town10HD_Opt_Other")

        self.assertTrue(world._load_map(wait_time=0))

        world.client.load_world.assert_called_once_with("Town10HD_Opt")
