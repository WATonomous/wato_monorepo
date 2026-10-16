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
"""Scenario that loads the converted oval OpenDRIVE map."""

from carla_scenarios.scenario_base import ScenarioBase

try:
    import carla
except ImportError:
    carla = None


class OvalScenario(ScenarioBase):
    """Load oval.xodr and spawn the ego vehicle on a driving lane."""

    XODR_FILENAME = "oval.xodr"

    def get_name(self) -> str:
        return "Oval OpenDRIVE Map"

    def get_description(self) -> str:
        return "Ego vehicle on the Lanelet2-derived oval OpenDRIVE map"

    def initialize(self, client: "carla.Client") -> bool:
        return self._initialize_carla(client)

    def setup(self) -> bool:
        if self.world is None:
            return False

        try:
            self._load_xodr_map(self.XODR_FILENAME)

            carla_map = self.world.get_map()
            spawn_points = carla_map.get_spawn_points()
            if spawn_points:
                spawn_point = spawn_points[0]
            else:
                waypoints = carla_map.generate_waypoints(2.0)
                if not waypoints:
                    self._log(
                        "oval.xodr loaded, but it has no driving-lane waypoints",
                        "error",
                    )
                    return False
                spawn_point = waypoints[0].transform
                spawn_point.location.z += 0.5

            if not self.spawn_ego_vehicle(spawn_point):
                return False

            self.world.set_weather(carla.WeatherParameters.ClearNoon)
            self._log(f"Setup complete: {self.get_name()}")
            return True
        except Exception as error:
            self._log(f"Failed to setup {self.get_name()}: {error}", "error")
            return False
