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

import random
from abc import ABC, abstractmethod
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

    def _log(self, msg: str, level: str = "info") -> None:
        """Log using ROS logger if available, otherwise print."""
        if self.logger:
            getattr(self.logger, level)(msg)
        else:
            print(msg, flush=True)

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

        results = self.client.apply_batch_sync(batch, True)

        # Collect spawned vehicles
        vehicles = []
        spawn_errors = 0
        for result in results:
            if not result.error:
                actor = self.world.get_actor(result.actor_id)
                if actor:
                    vehicles.append(actor)
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

        results = self.client.apply_batch_sync(batch, True)

        # Collect spawned walkers
        walkers_list = []
        final_speeds = []
        for i, result in enumerate(results):
            if not result.error:
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

        results = self.client.apply_batch_sync(batch, True)

        for i, result in enumerate(results):
            if not result.error:
                walkers_list[i]["con"] = result.actor_id
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

        # Wait for tick before starting controllers
        settings = self.world.get_settings()
        if settings.synchronous_mode:
            self.world.tick()
        else:
            self.world.wait_for_tick()

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
