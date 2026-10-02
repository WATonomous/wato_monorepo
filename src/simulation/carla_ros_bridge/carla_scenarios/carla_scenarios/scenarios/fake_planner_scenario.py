"""Ego-only test environment. The carla_fake_planner ROS node publishes trajectories."""

import math
import carla

from carla_common.lanelet_track import LaneletTrack
from carla_scenarios.scenario_base import ScenarioBase


class FakePlannerScenario(ScenarioBase):
    def get_name(self):
        return "Fake Planner"

    def get_description(self):
        return "Lane following, snake, and braking tests on the configured track"

    def initialize(self, client):
        return self._initialize_carla(client)

    def setup(self):
        if self.map_bundle is None:
            raise ValueError("fake planner scenario requires a map bundle")
        self._load_map()
        bundle = self.map_bundle
        track = LaneletTrack(bundle.osm_path, bundle.origin["lat"], bundle.origin["lon"])
        spawn = bundle.data["ego_spawn"]
        lane = track.lanes[int(spawn["lanelet_id"])]
        x, y, yaw = lane.center.at(float(spawn.get("distance_m", 0.0)))
        transform = carla.Transform(carla.Location(x=x, y=-y, z=float(spawn.get("z", 0.4))),
                                    carla.Rotation(yaw=-math.degrees(yaw)))
        ego = self.spawn_ego_vehicle(transform)
        if ego is None:
            return False
        ego.set_autopilot(False)
        ego.apply_control(carla.VehicleControl(brake=1.0))
        self.world.set_weather(carla.WeatherParameters.ClearNoon)
        return True
