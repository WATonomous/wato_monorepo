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
"""Heavy traffic scenario - many vehicles and pedestrians."""

from carla_scenarios.scenario_base import ScenarioBase

try:
    import carla
except ImportError:
    carla = None


class HeavyTrafficScenario(ScenarioBase):
    """Heavy traffic scenario with many NPC vehicles and pedestrians."""

    # Configuration
    NUM_VEHICLES = 50
    NUM_PEDESTRIANS = 100
    SPAWN_POINT_INDEX = 25
    PEDESTRIAN_CROSS_FACTOR = 0.2  # 20% chance to cross roads

    def get_name(self) -> str:
        return "Heavy Traffic"

    def get_description(self) -> str:
        return "Dense traffic with many vehicles and pedestrians"

    def initialize(self, client: "carla.Client") -> bool:
        if carla is None:
            self._log("CARLA module not available", "error")
            return False

        try:
            self.client = client
            self.world = client.get_world()
            return True
        except Exception as e:
            self._log(f"Failed to initialize: {e}", "error")
            return False

    def setup(self) -> bool:
        if self.world is None:
            return False

        try:
            import time

            # Load Town10HD if needed
            current_map = self.world.get_map().name
            if "Town10HD" not in current_map:
                self._log(f"Loading Town10HD map (current: {current_map})...")
                self.client.load_world("Town10HD")
                time.sleep(2.0)
                self.world = self.client.get_world()

            # Get spawn points
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                self._log("No spawn points available", "error")
                return False

            # Spawn ego vehicle
            spawn_index = min(self.SPAWN_POINT_INDEX, len(spawn_points) - 1)
            if not self.spawn_ego_vehicle(spawn_points[spawn_index]):
                return False

            # Set weather
            self.world.set_weather(carla.WeatherParameters.CloudyNoon)

            # Spawn NPCs
            npc_spawn_points = [
                sp for i, sp in enumerate(spawn_points) if i != spawn_index
            ]
            self.spawn_vehicles(self.NUM_VEHICLES, npc_spawn_points)
            self.spawn_pedestrians(self.NUM_PEDESTRIANS, self.PEDESTRIAN_CROSS_FACTOR)

            self._log(f"Setup complete: {self.get_name()}")
            return True

        except Exception as e:
            self._log(f"Failed to setup: {e}", "error")
            import traceback

            traceback.print_exc()
            return False

    def execute(self) -> None:
        pass
