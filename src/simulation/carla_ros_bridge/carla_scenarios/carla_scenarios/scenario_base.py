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
"""Base class for CARLA scenario plugins."""

import hashlib
import math
import random
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Optional, List

try:
    import carla
except ImportError:
    carla = None  # Will be checked at runtime


class ScenarioBase(ABC):
    """Abstract base class for scenario plugins."""

    def __init__(self):
        self.client: Optional["carla.Client"] = None
        self.world: Optional["carla.World"] = None
        self.logger = None  # Set by scenario server
        self.xodr_map_directory: Optional[Path] = None
        self.owned_actors = []
        self.advance_world = None

    def _log(self, msg: str, level: str = "info") -> None:
        """Log using ROS logger if available, otherwise print."""
        if self.logger:
            getattr(self.logger, level)(msg)
        else:
            print(msg, flush=True)

    # -------------------------------------------------------------------------
    # Common Implementation Helpers
    # -------------------------------------------------------------------------

    def _initialize_carla(self, client: "carla.Client") -> bool:
        """
        Standard CARLA initialization.

        Args:
            client: CARLA client instance

        Returns:
            True if initialization successful
        """
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

    def _load_map(self, map_name: str = "Town10HD_Opt", wait_time: float = 2.0) -> bool:
        """
        Load a map if not already loaded.

        Args:
            map_name: Name of the map to load
            wait_time: Time to wait after loading (seconds)

        Returns:
            True if map is ready
        """
        import time

        current_map = self.world.get_map().name
        current_map_id = current_map.removeprefix("/Game/")
        requested_map_id = map_name.removeprefix("/Game/")
        if "/" not in requested_map_id:
            current_map_id = current_map_id.rsplit("/", 1)[-1]
        if current_map_id != requested_map_id:
            self._log(f"Loading {map_name} map (current: {current_map})...")
            self.client.load_world(map_name)
            time.sleep(wait_time)
            self.world = self.client.get_world()
        return True

    def _load_xodr_map(self, xodr_file: str) -> bool:
        """
        Generate a CARLA world from an OpenDRIVE file.

        Relative paths are resolved from the scenario server's
        xodr_map_directory parameter.

        Args:
            xodr_file: OpenDRIVE filename or absolute path

        Returns:
            True when CARLA generated the world successfully

        Raises:
            RuntimeError: If CARLA is unavailable or rejects the map
            ValueError: If the path or OpenDRIVE document is invalid
        """
        if self.client is None:
            raise RuntimeError("CARLA client is not initialized")

        xodr_path = Path(xodr_file)
        if not xodr_path.is_absolute():
            if self.xodr_map_directory is None:
                raise ValueError("xodr_map_directory is not configured")
            xodr_path = self.xodr_map_directory / xodr_path
        xodr_path = xodr_path.resolve()

        if xodr_path.suffix.lower() != ".xodr":
            raise ValueError(f"OpenDRIVE map must have a .xodr extension: {xodr_path}")
        if not xodr_path.is_file():
            raise ValueError(f"OpenDRIVE map does not exist: {xodr_path}")

        try:
            xodr_data = xodr_path.read_text(encoding="utf-8")
        except OSError as error:
            raise ValueError(
                f"OpenDRIVE map could not be read: {xodr_path}: {error}"
            ) from error

        try:
            root = ET.fromstring(xodr_data)
        except ET.ParseError as error:
            raise ValueError(
                f"OpenDRIVE map is not valid XML: {xodr_path}: {error}"
            ) from error

        root_name = root.tag.rsplit("}", 1)[-1]
        if root_name != "OpenDRIVE" or not root.findall(".//{*}road"):
            raise ValueError(
                f"OpenDRIVE map must contain an OpenDRIVE root and at least one road: "
                f"{xodr_path}"
            )

        self._log(f"Generating CARLA world from {xodr_path}...")
        try:
            self.world = self.client.generate_opendrive_world(
                xodr_data,
                carla.OpendriveGenerationParameters(
                    wall_height=0.0,
                    enable_mesh_visibility=True,
                    # The oval scenario has no pedestrians. CARLA 0.10 can
                    # remain busy indefinitely building OpenDriveMap.obj when
                    # pedestrian navigation generation is left enabled.
                    enable_pedestrian_navigation=False,
                ),
            )
        except Exception as error:
            raise RuntimeError(
                f"CARLA failed to generate a world from {xodr_path}: {error}"
            ) from error

        if self.world is None:
            raise RuntimeError(
                f"CARLA did not return a world for OpenDRIVE map: {xodr_path}"
            )

        return True

    # -------------------------------------------------------------------------
    # Shared Spawn Utilities
    # -------------------------------------------------------------------------

    def spawn_ego_vehicle(
        self,
        spawn_point: "carla.Transform",
        blueprint_filter: str = "vehicle.mini.cooper",
        role_name: str = "ego_vehicle",
    ) -> Optional["carla.Vehicle"]:
        """
        Spawn the ego vehicle.

        Args:
            spawn_point: Where to spawn the vehicle
            blueprint_filter: Blueprint filter string
            role_name: Role name attribute for the vehicle

        Returns:
            The spawned vehicle actor, or None on failure
        """
        blueprint_library = self.world.get_blueprint_library()
        vehicle_bp = blueprint_library.filter(blueprint_filter)[0]
        vehicle_bp.set_attribute("role_name", role_name)

        self._log(f"Spawning ego vehicle at {spawn_point.location}")
        ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)

        if ego_vehicle is None:
            self._log("Failed to spawn ego vehicle", "error")
            return None

        self.owned_actors.append(ego_vehicle)
        return ego_vehicle

    def spawn_vehicles(
        self,
        count: int,
        spawn_points: List["carla.Transform"],
        autopilot: bool = True,
    ) -> List["carla.Vehicle"]:
        """
        Spawn NPC vehicles.

        Args:
            count: Number of vehicles to spawn
            spawn_points: Available spawn points
            autopilot: Whether to enable autopilot

        Returns:
            List of spawned vehicle actors
        """
        self._log(f"Spawning {count} NPC vehicles...")

        if not spawn_points:
            self._log("No spawn points available for vehicles", "warn")
            return []

        blueprint_library = self.world.get_blueprint_library()
        vehicle_blueprints = list(blueprint_library.filter("vehicle.*"))

        if not vehicle_blueprints:
            self._log("No vehicle blueprints found", "warn")
            return []

        # Shuffle and limit spawn points
        points = list(spawn_points)
        random.shuffle(points)
        num_to_spawn = min(count, len(points))

        # Batch spawn
        batch = []
        for i in range(num_to_spawn):
            blueprint = random.choice(vehicle_blueprints)

            if blueprint.has_attribute("color"):
                color = random.choice(
                    blueprint.get_attribute("color").recommended_values
                )
                blueprint.set_attribute("color", color)

            blueprint.set_attribute("role_name", "npc")
            batch.append(carla.command.SpawnActor(blueprint, points[i]))

        results = self.client.apply_batch_sync(batch, False)

        # Collect spawned vehicles
        vehicles = []
        spawn_errors = 0
        for result in results:
            if not result.error:
                actor = self.world.get_actor(result.actor_id)
                if actor:
                    vehicles.append(actor)
                    self.owned_actors.append(actor)
                    if autopilot:
                        actor.set_autopilot(True)
            else:
                spawn_errors += 1

        self._log(f"Spawned {len(vehicles)} vehicles, {spawn_errors} failed")
        return vehicles

    def spawn_pedestrians(
        self,
        count: int,
        cross_roads_factor: float = 0.0,
    ) -> List["carla.Walker"]:
        """
        Spawn NPC pedestrians with AI controllers.

        Args:
            count: Number of pedestrians to spawn
            cross_roads_factor: Probability of pedestrians crossing roads (0.0-1.0)

        Returns:
            List of spawned walker actors
        """
        self._log(f"Spawning {count} pedestrians...")

        blueprint_library = self.world.get_blueprint_library()
        walker_blueprints = list(blueprint_library.filter("walker.pedestrian.*"))

        if not walker_blueprints:
            self._log("No walker blueprints found", "warn")
            return []

        # Get spawn points from navigation mesh
        spawn_points = []
        for _ in range(count * 2):  # Try more locations than needed
            loc = self.world.get_random_location_from_navigation()
            if loc is not None:
                spawn_point = carla.Transform()
                spawn_point.location = loc
                spawn_point.location.z += 2  # Offset to avoid collision
                spawn_points.append(spawn_point)

        if not spawn_points:
            self._log("No valid pedestrian spawn locations found", "warn")
            return []

        num_to_spawn = min(count, len(spawn_points))

        # Batch spawn walkers
        batch = []
        walker_speeds = []
        for i in range(num_to_spawn):
            blueprint = random.choice(walker_blueprints)

            if blueprint.has_attribute("is_invincible"):
                blueprint.set_attribute("is_invincible", "false")

            if blueprint.has_attribute("speed"):
                walker_speeds.append(
                    float(blueprint.get_attribute("speed").recommended_values[1])
                )
            else:
                walker_speeds.append(1.4)  # Default walking speed

            batch.append(carla.command.SpawnActor(blueprint, spawn_points[i]))

        results = self.client.apply_batch_sync(batch, False)

        # Collect spawned walkers
        walkers_list = []
        final_speeds = []
        for i, result in enumerate(results):
            if not result.error:
                self.owned_actors.append(self.world.get_actor(result.actor_id))
                walkers_list.append({"id": result.actor_id})
                final_speeds.append(walker_speeds[i])

        if not walkers_list:
            self._log("No walkers spawned", "warn")
            return []

        # Spawn AI controllers
        walker_controller_bp = blueprint_library.find("controller.ai.walker")
        batch = []
        for walker_info in walkers_list:
            batch.append(
                carla.command.SpawnActor(
                    walker_controller_bp, carla.Transform(), walker_info["id"]
                )
            )

        results = self.client.apply_batch_sync(batch, False)

        for i, result in enumerate(results):
            if not result.error:
                walkers_list[i]["con"] = result.actor_id
                self.owned_actors.append(self.world.get_actor(result.actor_id))
            else:
                walkers_list[i]["con"] = None

        # Build list of valid walker-controller pairs
        all_ids = []
        walker_speed_list = []
        for i, walker_info in enumerate(walkers_list):
            if walker_info.get("con") is not None:
                all_ids.append(walker_info["con"])
                all_ids.append(walker_info["id"])
                walker_speed_list.append(final_speeds[i])

        if not all_ids:
            self._log("No valid walker-controller pairs created", "warn")
            return []

        all_actors = self.world.get_actors(all_ids)

        # Wait for next tick before starting controllers
        if self.advance_world is None:
            raise RuntimeError(
                "Scenario setup requires the scenario server tick authority"
            )
        self.advance_world()

        # Set crossing factor
        self.world.set_pedestrians_cross_factor(cross_roads_factor)

        # Start controllers
        walkers = []
        for i in range(0, len(all_ids), 2):
            controller = all_actors[i]
            walker = all_actors[i + 1]
            walkers.append(walker)

            controller.start()
            controller.go_to_location(self.world.get_random_location_from_navigation())
            controller.set_max_speed(float(walker_speed_list[i // 2]))

            # Re-enable collisions and physics for LiDAR detection
            # (CARLA disables these for AI walkers as optimization, breaking LiDAR)
            # See: https://github.com/carla-simulator/carla/issues/7092
            walker.set_collisions(True)
            walker.set_simulate_physics(True)

        self._log(f"Spawned {len(walkers)} pedestrians with AI controllers")
        return walkers

    # -------------------------------------------------------------------------
    # Abstract Methods
    # -------------------------------------------------------------------------

    @abstractmethod
    def get_name(self) -> str:
        """Return scenario name."""
        pass

    @abstractmethod
    def get_description(self) -> str:
        """Return scenario description."""
        pass

    @abstractmethod
    def initialize(self, client: "carla.Client") -> bool:
        """
        Initialize scenario with CARLA client.

        Args:
            client: CARLA client instance

        Returns:
            True if initialization successful, False otherwise
        """
        pass

    @abstractmethod
    def setup(self) -> bool:
        """
        Set up CARLA world for this scenario.

        Returns:
            True if setup successful, False otherwise
        """
        pass

    def execute(self) -> None:
        """Execute scenario logic (called periodically). Override if needed."""
        pass

    def cleanup(self) -> None:
        """Release only owned actors, retaining failed handles for a close retry."""
        remaining = []
        for actor in reversed(self.owned_actors):
            if actor is None or not actor.is_alive:
                continue
            try:
                if actor.type_id.startswith(("controller.", "sensor.")):
                    actor.stop()
            except RuntimeError as error:
                self._log(f"Could not stop owned actor {actor.id}: {error}", "warn")
            try:
                if not actor.destroy():
                    remaining.append(actor)
            except RuntimeError as error:
                remaining.append(actor)
                self._log(f"Could not destroy owned actor {actor.id}: {error}", "warn")
        self.owned_actors = list(reversed(remaining))
        if remaining:
            raise RuntimeError(
                f"Failed to destroy {len(remaining)} owned actors; retry close"
            )


class ManifestScenario(ScenarioBase):
    """Reviewed YAML playground primitive, using the same converted map as ROS.

    Geometry returned to the runner is always in the ROS map frame. CARLA uses
    left-handed coordinates, so both y and heading change sign at this boundary.
    """

    def __init__(self, definition, xodr_path):
        super().__init__()
        self.definition = definition
        self.xodr_path = xodr_path
        self.ego_vehicle = None
        self.parked_vehicle = None
        self.geometry = {}
        self.held = True

    def get_name(self):
        return self.definition["id"]

    def get_description(self):
        return f"YAML playground {self.get_name()}"

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

    def setup(self):
        self._load_xodr_map(str(self.xodr_path))
        waypoint = self._spawn_waypoint()
        spawn = waypoint.transform
        spawn.location.z += float(
            self.definition.get("spawn", {}).get("z_offset_m", 0.4)
        )
        self.ego_vehicle = self.spawn_ego_vehicle(
            spawn, self.definition.get("ego_blueprint", "vehicle.mini.cooper")
        )
        self.set_held(True)
        parked = self.definition.get("parked")
        if parked:
            ahead = waypoint.next(float(parked["distance_m"]))
            if len(ahead) != 1:
                raise ValueError(
                    "Parked vehicle distance leaves the selected continuous lane"
                )
            parked_spawn = ahead[0].transform
            parked_spawn.location.z += 0.4
            self.parked_vehicle = self.spawn_ego_vehicle(
                parked_spawn,
                parked.get("blueprint", "vehicle.mini.cooper"),
                "parked_car",
            )
            self.parked_vehicle.set_simulate_physics(False)
            self.parked_vehicle.apply_control(
                carla.VehicleControl(brake=1.0, hand_brake=True)
            )
        goal_distance = float(self.definition.get("goal_distance_m", 0.0))
        goal_waypoints = (
            waypoint.next(goal_distance) if goal_distance > 0.0 else [waypoint]
        )
        if len(goal_waypoints) != 1:
            raise ValueError("Goal leaves the selected continuous lane")
        goal = goal_waypoints[0].transform.location
        heading = math.radians(spawn.rotation.yaw)
        self.world.set_weather(carla.WeatherParameters.ClearNoon)
        self.geometry = {
            "frame_id": "map",
            "ego": self._actor_geometry(self.ego_vehicle),
            "parked": self._actor_geometry(self.parked_vehicle)
            if self.parked_vehicle
            else None,
            "lane": {
                "x": spawn.location.x,
                "y": -spawn.location.y,
                "dx": math.cos(heading),
                "dy": -math.sin(heading),
            },
            "goal": {"x": goal.x, "y": -goal.y, "z": 0.0},
            "spawn": {
                "road_id": waypoint.road_id,
                "lane_id": waypoint.lane_id,
                "s": waypoint.s,
                "yaw_deg": spawn.rotation.yaw,
            },
            "map": {
                "path": str(self.xodr_path),
                "sha256": hashlib.sha256(self.xodr_path.read_bytes()).hexdigest(),
            },
            "seed": self.definition.get("seed", 42),
            "reset_policy": "reload_world",
        }
        return True

    def set_held(self, held):
        """Hold initialization stationary while bridge nodes collect fresh samples."""
        self.held = held
        if self.ego_vehicle:
            self.ego_vehicle.set_simulate_physics(not held)
            self.ego_vehicle.apply_control(carla.VehicleControl(brake=1.0))
