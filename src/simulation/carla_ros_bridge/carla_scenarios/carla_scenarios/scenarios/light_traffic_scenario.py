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
"""Light traffic scenario - few vehicles and pedestrians."""

from carla_scenarios.scenario_base import ScenarioBase

try:
    import carla
except ImportError:
    carla = None


class LightTrafficScenario(ScenarioBase):
    """Light traffic scenario with minimal NPC vehicles and pedestrians."""

    NUM_VEHICLES = 5
    NUM_PEDESTRIANS = 10
    SPAWN_POINT_INDEX = 5

    def get_name(self) -> str:
        return "Light Traffic"

    def get_description(self) -> str:
        return "Sparse traffic with few vehicles and pedestrians"

    def initialize(self, client: "carla.Client") -> bool:
        return self._initialize_carla(client)

    def setup(self) -> bool:
        if self.world is None:
            return False

        try:
            self._load_map("Town10HD")

            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                self._log("No spawn points available", "error")
                return False

            spawn_index = min(self.SPAWN_POINT_INDEX, len(spawn_points) - 1)
            if not self.spawn_ego_vehicle(spawn_points[spawn_index]):
                return False

            self.world.set_weather(carla.WeatherParameters.ClearSunset)

            npc_spawn_points = [
                sp for i, sp in enumerate(spawn_points) if i != spawn_index
            ]
            self.spawn_vehicles(self.NUM_VEHICLES, npc_spawn_points)
            self.spawn_pedestrians(self.NUM_PEDESTRIANS)

            self._log(f"Setup complete: {self.get_name()}")
            return True

        except Exception as e:
            self._log(f"Failed to setup: {e}", "error")
            return False
