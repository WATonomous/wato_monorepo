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
"""Stop sign scenario that begins the scenario right at a stop sign."""

from carla_scenarios.scenario_base import ScenarioBase

try:
    import carla
except ImportError:
    carla = None


class StopSignScenario(ScenarioBase):
    """Mixed-traffic scenario with stop signs for CARLA experiments."""

    NUM_VEHICLES = 10
    NUM_PEDESTRIANS = 20
    NUM_TRAFFIC_ENTITIES = 8
    STOP_SIGN_APPROACH_DISTANCE = 6.0  # Metres upstream of the trigger centre.
    EGO_SPAWN_HEIGHT = 0.3
    EGO_SPAWN_CLEARANCE = 8.0
    PEDESTRIAN_CROSS_FACTOR = 0.1
    TRAFFIC_PROP_FILTERS = (
        "static.prop.trafficcone*",
        "static.prop.streetbarrier*",
    )

    def __init__(self):
        super().__init__()
        self.traffic_entities = []

    def get_name(self) -> str:
        return "Stop Sign Scenario"

    def get_description(self) -> str:
        return "Mixed traffic with NPC vehicles, pedestrians, and traffic props"

    def initialize(self, client: "carla.Client") -> bool:
        return self._initialize_carla(client)

    def _get_ego_spawn_point(self, road_map: "carla.Map") -> "carla.Transform | None":
        """Find a lane-aligned spawn just before a stop sign trigger."""
        stop_signs = self.world.get_actors().filter("traffic.stop*")
        for sign in sorted(stop_signs, key=lambda actor: actor.id):
            trigger_location = sign.get_transform().transform(
                sign.trigger_volume.location
            )
            stop_waypoint = road_map.get_waypoint(trigger_location)
            if stop_waypoint is None:
                continue

            for approach in stop_waypoint.previous(self.STOP_SIGN_APPROACH_DISTANCE):
                if approach.is_junction:
                    continue
                spawn_point = approach.transform
                spawn_point.location.z += self.EGO_SPAWN_HEIGHT
                return spawn_point

        return None

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
            self._load_map("Town10HD_Opt")

            road_map = self.world.get_map()
            spawn_points = road_map.get_spawn_points()
            if not spawn_points:
                self._log("No spawn points available", "error")
                return False

            ego_spawn_point = self._get_ego_spawn_point(road_map)
            if ego_spawn_point is None:
                self._log(
                    "No suitable stop sign approach found for the ego vehicle", "error"
                )
                return False
            if not self.spawn_ego_vehicle(ego_spawn_point):
                return False

            self.world.set_weather(carla.WeatherParameters.ClearNoon)

            npc_spawn_points = [
                point
                for point in spawn_points
                if point.location.distance(ego_spawn_point.location)
                >= self.EGO_SPAWN_CLEARANCE
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
