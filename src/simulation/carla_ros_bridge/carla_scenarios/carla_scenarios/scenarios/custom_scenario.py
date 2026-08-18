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
"""Custom scenario with vehicles, pedestrians, and traffic props."""

from carla_scenarios.scenario_base import ScenarioBase

try:
    import carla
except ImportError:
    carla = None


class CustomScenario(ScenarioBase):
    """Customizable mixed-traffic scenario for CARLA experiments."""

    NUM_VEHICLES = 10
    NUM_PEDESTRIANS = 20
    NUM_TRAFFIC_ENTITIES = 8
    SPAWN_POINT_INDEX = 10
    PEDESTRIAN_CROSS_FACTOR = 0.1
    TRAFFIC_PROP_FILTERS = (
        "static.prop.trafficcone*",
        "static.prop.streetbarrier*",
    )

    def __init__(self):
        super().__init__()
        self.traffic_entities = []

    def get_name(self) -> str:
        return "Custom Scenario"

    def get_description(self) -> str:
        return "Mixed traffic with NPC vehicles, pedestrians, and traffic props"

    def initialize(self, client: "carla.Client") -> bool:
        return self._initialize_carla(client)

    def _spawn_traffic_entities(self, spawn_points: list["carla.Transform"]) -> int:
        """Spawn cones and barriers at reserved vehicle spawn points."""
        blueprint_library = self.world.get_blueprint_library()
        traffic_blueprints = []
        for blueprint_filter in self.TRAFFIC_PROP_FILTERS:
            traffic_blueprints.extend(blueprint_library.filter(blueprint_filter))

        if not traffic_blueprints:
            self._log("No traffic-prop blueprints found", "warn")
            return 0

        spawned = 0
        for index, transform in enumerate(spawn_points[: self.NUM_TRAFFIC_ENTITIES]):
            blueprint = traffic_blueprints[index % len(traffic_blueprints)]
            actor = self.world.try_spawn_actor(blueprint, transform)
            if actor is not None:
                self.traffic_entities.append(actor)
                spawned += 1

        self._log(f"Spawned {spawned} traffic entities")
        return spawned

    def cleanup(self) -> None:
        """Destroy only the static traffic props created by this scenario."""
        for actor in self.traffic_entities:
            try:
                actor.destroy()
            except Exception:
                pass
        self.traffic_entities = []

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

            self.world.set_weather(carla.WeatherParameters.ClearNoon)

            npc_spawn_points = [
                point for i, point in enumerate(spawn_points) if i != spawn_index
            ]
            traffic_spawn_points = npc_spawn_points[: self.NUM_TRAFFIC_ENTITIES]
            vehicle_spawn_points = npc_spawn_points[self.NUM_TRAFFIC_ENTITIES :]

            self._spawn_traffic_entities(traffic_spawn_points)
            self.spawn_vehicles(self.NUM_VEHICLES, vehicle_spawn_points)
            self.spawn_pedestrians(self.NUM_PEDESTRIANS, self.PEDESTRIAN_CROSS_FACTOR)

            self._log(f"Setup complete: {self.get_name()}")
            return True

        except Exception as e:
            self._log(f"Failed to setup: {e}", "error")
            return False
