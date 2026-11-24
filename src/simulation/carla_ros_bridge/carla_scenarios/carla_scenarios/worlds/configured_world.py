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
"""Shared town and OpenDRIVE world setup, including scored-test actor fixtures."""

import hashlib
import math

import carla

from carla_scenarios.scenario_base import ScenarioBase


class ConfiguredWorld(ScenarioBase):
    """Reviewed YAML playground primitive, using the same converted map as ROS.

    Geometry returned to the runner is always in the ROS map frame. CARLA uses
    left-handed coordinates, so both y and heading change sign at this boundary.
    """

    def __init__(self, definition, xodr_path=None, *, held=False):
        super().__init__()
        self.definition = definition
        self.xodr_path = xodr_path
        self.ego_vehicle = None
        self.target_vehicle = None
        self.geometry = {}
        self.held = held

    def get_name(self):
        return self.definition["id"]

    def get_description(self):
        return self.definition.get("description", self.get_name())

    def initialize(self, client):
        return self._initialize_carla(client)

    @staticmethod
    def _actor_geometry(actor):
        transform = actor.get_transform()
        bounds = actor.bounding_box
        # A blueprint's bounding box need not be centered at the actor origin.
        return {
            "x": transform.location.x,
            "y": -transform.location.y,
            "z": transform.location.z,
            "yaw": -math.radians(transform.rotation.yaw),
            "length_m": 2.0 * bounds.extent.x,
            "width_m": 2.0 * bounds.extent.y,
            "center_offset_x_m": bounds.location.x,
            "center_offset_y_m": -bounds.location.y,
            "actor_id": actor.id,
            "blueprint": actor.type_id,
        }

    def _spawn_waypoint(self, carla_map=None):
        spawn = self.definition.get("spawn", {})
        carla_map = carla_map or self.world.get_map()
        required = float(self.definition.get("required_straight_m", 0.0))
        margin = float(spawn.get("s_margin_m", 10.0))
        if spawn.get("straight_start"):
            # Fixed map content gives the same ordered candidates on every run.
            # Verify a continuous lane rather than assuming a road ID or heading.
            candidates = sorted(
                carla_map.generate_waypoints(2.0),
                key=lambda wp: (wp.road_id, wp.lane_id, wp.s),
            )
            for candidate in candidates:
                if candidate.is_junction:
                    continue
                advanced = candidate.next(margin)
                if len(advanced) != 1:
                    continue
                waypoint = advanced[0]
                heading = math.radians(waypoint.transform.rotation.yaw)
                valid = True
                for distance in range(2, int(math.ceil(required)) + 2, 2):
                    ahead = waypoint.next(float(distance))
                    if len(ahead) != 1 or ahead[0].is_junction:
                        valid = False
                        break
                    transform = ahead[0].transform
                    angle = math.radians(transform.rotation.yaw) - heading
                    angle = math.atan2(math.sin(angle), math.cos(angle))
                    lateral = -(
                        transform.location.x - waypoint.transform.location.x
                    ) * math.sin(heading) + (
                        transform.location.y - waypoint.transform.location.y
                    ) * math.cos(heading)
                    if abs(angle) > math.radians(1.0) or abs(lateral) > 0.25:
                        valid = False
                        break
                if valid:
                    return waypoint
            raise ValueError(
                f"{self.xodr_path}: no continuous straight driving lane with "
                f"{required} m runout; adjust the playground geometry/map"
            )
        if {"road_id", "lane_id", "s"}.issubset(spawn):
            waypoint = carla_map.get_waypoint_xodr(
                int(spawn["road_id"]), int(spawn["lane_id"]), float(spawn["s"])
            )
            if waypoint is None:
                raise ValueError(f"Spawn road/lane/s does not exist: {spawn}")
            return waypoint
        candidates = sorted(
            carla_map.generate_waypoints(2.0),
            key=lambda wp: (wp.road_id, wp.lane_id, wp.s),
        )
        if not candidates:
            raise ValueError(f"{self.xodr_path}: no driving-lane waypoint")
        return candidates[0]

    @staticmethod
    def _copy_transform(transform, z_offset=0.0):
        return carla.Transform(
            carla.Location(
                x=transform.location.x,
                y=transform.location.y,
                z=transform.location.z + z_offset,
            ),
            carla.Rotation(
                pitch=transform.rotation.pitch,
                yaw=transform.rotation.yaw,
                roll=transform.rotation.roll,
            ),
        )

    def resolve_pose(self, pose, waypoint=None):
        """Convert ROS map poses or lane distances to CARLA transforms."""
        if pose.get("frame", "map") == "map":
            return carla.Transform(
                carla.Location(
                    x=float(pose["x"]), y=-float(pose["y"]), z=float(pose.get("z", 0.4))
                ),
                carla.Rotation(yaw=-float(pose.get("yaw_deg", 0.0))),
            )
        if waypoint is None:
            raise ValueError(
                "Lane-relative actor/goal poses require a lane waypoint ego spawn"
            )
        distance = float(pose["distance_m"])
        ahead = [waypoint] if distance == 0 else waypoint.next(distance)
        if len(ahead) != 1:
            raise ValueError(
                f"Actor or goal at {distance} m leaves the continuous lane"
            )
        return self._copy_transform(
            ahead[0].transform, float(pose.get("z_offset_m", 0.4))
        )

    def validate_fixtures(self, carla_map):
        settings = self.definition.get("spawn", {})
        waypoint = (
            self._spawn_waypoint(carla_map)
            if settings.get("straight_start")
            or {"road_id", "lane_id", "s"} <= settings.keys()
            else None
        )
        if "pose" in settings:
            self.resolve_pose(settings["pose"])
        for actor in self.definition.get("actors", []):
            self.resolve_pose(actor["pose"], waypoint)
        if "goal" in self.definition:
            self.resolve_pose(self.definition["goal"], waypoint)

    def setup(self):
        if self.xodr_path is not None:
            self._load_xodr_map(str(self.xodr_path))
        else:
            self._load_map(self.definition["map"]["town"])
        carla_map = self.world.get_map()
        settings = self.definition.get("spawn", {})
        points = carla_map.get_spawn_points()
        waypoint = None
        if "pose" in settings:
            spawn = self.resolve_pose(settings["pose"])
        elif (
            settings.get("straight_start")
            or {"road_id", "lane_id", "s"} <= settings.keys()
        ):
            waypoint = self._spawn_waypoint(carla_map)
            spawn = waypoint.transform
            spawn.location.z += float(settings.get("z_offset_m", 0.4))
        elif points:
            spawn = points[min(settings.get("index", 0), len(points) - 1)]
        elif settings.get("fallback_waypoint", False):
            candidates = carla_map.generate_waypoints(2.0)
            if not candidates:
                raise ValueError("Map has no driving-lane waypoints")
            waypoint = candidates[0]
            spawn = waypoint.transform
            spawn.location.z += float(settings.get("fallback_z_offset_m", 0.5))
        else:
            raise ValueError("Map has no spawn points")
        self.ego_vehicle = self.spawn_ego_vehicle(
            spawn, self.definition.get("ego_blueprint", "vehicle.mini.cooper")
        )
        if self.ego_vehicle is None:
            raise RuntimeError("Could not spawn ego vehicle")
        self.set_held(self.held)
        self._spawn_traffic(points, spawn)
        actors = {}
        for specification in self.definition.get("actors", []):
            actor = self.spawn_ego_vehicle(
                self.resolve_pose(specification["pose"], waypoint),
                specification["blueprint"],
                specification["id"],
            )
            if actor is None:
                raise RuntimeError(f"Could not spawn actor {specification['id']}")
            if specification.get("static", True):
                actor.set_simulate_physics(False)
                actor.apply_control(carla.VehicleControl(brake=1.0, hand_brake=True))
            actors[specification["id"]] = actor
        target = self.definition.get("evaluation_actor")
        if target:
            self.target_vehicle = actors[target]
        goal = None
        if "goal" in self.definition:
            goal = self.resolve_pose(self.definition["goal"], waypoint).location
        heading = math.radians(spawn.rotation.yaw)
        self.world.set_weather(
            getattr(
                carla.WeatherParameters, self.definition.get("weather", "ClearNoon")
            )
        )
        self.geometry = {
            "frame_id": "map",
            "ego": self._actor_geometry(self.ego_vehicle),
            "actors": {
                name: self._actor_geometry(actor) for name, actor in actors.items()
            },
            "target_actor": self._actor_geometry(self.target_vehicle)
            if self.target_vehicle
            else None,
            "lane": {
                "x": spawn.location.x,
                "y": -spawn.location.y,
                "dx": math.cos(heading),
                "dy": -math.sin(heading),
            },
            "goal": {"x": goal.x, "y": -goal.y, "z": 0.0} if goal else None,
            "spawn": {
                "road_id": waypoint.road_id if waypoint else None,
                "lane_id": waypoint.lane_id if waypoint else None,
                "s": waypoint.s if waypoint else None,
                "yaw_deg": spawn.rotation.yaw,
            },
            "map": {
                "path": str(self.xodr_path)
                if self.xodr_path
                else self.definition["map"]["town"],
                "sha256": hashlib.sha256(self.xodr_path.read_bytes()).hexdigest()
                if self.xodr_path
                else None,
            },
            "seed": self.definition.get("seed", 42),
            "reset_policy": "reload_world",
        }
        return True

    def _spawn_traffic(self, spawn_points, ego_spawn):
        traffic = self.definition.get("traffic", {})
        available = [point for point in spawn_points if point is not ego_spawn]
        props = traffic.get("props", {})
        count = props.get("count", 0)
        # Reserve prop locations before handing the remaining points to traffic.
        library = self.world.get_blueprint_library()
        blueprints = [
            bp
            for pattern in props.get("blueprints", [])
            for bp in library.filter(pattern)
        ]
        if count and not blueprints:
            self._log("No traffic-prop blueprints found", "warn")
        for index, transform in enumerate(available[:count]):
            if blueprints:
                actor = self.world.try_spawn_actor(
                    blueprints[index % len(blueprints)], transform
                )
                if actor is not None:
                    self.owned_actors.append(actor)
        if traffic.get("vehicles", 0):
            self.spawn_vehicles(traffic["vehicles"], available[count:])
        if traffic.get("pedestrians", 0):
            self.spawn_pedestrians(
                traffic["pedestrians"], traffic.get("pedestrian_crossing_factor", 0.0)
            )

    def set_held(self, held):
        """Hold initialization stationary while bridge nodes collect fresh samples."""
        self.held = held
        if self.ego_vehicle:
            self.ego_vehicle.set_simulate_physics(not held)
            self.ego_vehicle.apply_control(carla.VehicleControl(brake=1.0))
