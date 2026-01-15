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
"""Default scenario implementation."""

from carla_scenarios.scenario_base import ScenarioBase

try:
    import carla
except ImportError:
    carla = None


class DefaultScenario(ScenarioBase):
    """Default scenario that spawns the ego vehicle."""

    def __init__(self):
        super().__init__()
        self.spawned_actors = []

    def get_name(self) -> str:
        """Return scenario name."""
        return "Default Ego Spawn"

    def get_description(self) -> str:
        """Return scenario description."""
        return "Basic scenario that spawns ego vehicle"

    def initialize(self, client: "carla.Client") -> bool:
        """Initialize scenario with CARLA client."""
        if carla is None:
            print("CARLA module not available")
            return False

        try:
            self.client = client
            self.world = client.get_world()
            return True
        except Exception as e:
            print(f"Failed to initialize default scenario: {e}")
            return False

    def setup(self) -> bool:
        """Set up CARLA world for default scenario."""
        if self.world is None:
            return False

        try:
            import time

            # Check if Town10HD is already loaded
            current_map = self.world.get_map().name
            if "Town10HD" not in current_map:
                print(f"Loading Town10HD map (current: {current_map})...")
                self.client.load_world("Town10HD")
                time.sleep(2.0)  # Wait for world to load
                self.world = self.client.get_world()
            else:
                print("Town10HD already loaded")

            # Get blueprint library
            blueprint_library = self.world.get_blueprint_library()

            # Get a vehicle blueprint (Mini Cooper)
            vehicle_bp = blueprint_library.filter("vehicle.mini.cooper")[0]

            # Set the role name for other nodes to find this vehicle
            vehicle_bp.set_attribute("role_name", "ego_vehicle")

            # Get a spawn point
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                print("No spawn points available in Town10")
                return False

            spawn_point = spawn_points[0]  # Use first spawn point

            # Spawn the ego vehicle
            print(f"Spawning ego vehicle at {spawn_point.location}")
            ego_vehicle = self.world.spawn_actor(vehicle_bp, spawn_point)

            if ego_vehicle is None:
                print("Failed to spawn ego vehicle")
                return False

            self.spawned_actors.append(ego_vehicle)

            # Position spectator camera behind and above the ego vehicle
            spectator = self.world.get_spectator()
            ego_transform = ego_vehicle.get_transform()
            spectator_transform = carla.Transform(
                ego_transform.location + carla.Location(x=-10, z=5),
                carla.Rotation(pitch=-15, yaw=ego_transform.rotation.yaw),
            )
            spectator.set_transform(spectator_transform)

            # Set weather to clear day
            weather = carla.WeatherParameters.ClearNoon
            self.world.set_weather(weather)

            print(f"Setup complete: {self.get_name()}")
            print(f"Ego vehicle spawned with ID: {ego_vehicle.id}")

            return True
        except Exception as e:
            print(f"Failed to setup default scenario: {e}")
            import traceback

            traceback.print_exc()
            return False

    def execute(self) -> None:
        """Execute scenario logic."""
        pass

    def cleanup(self) -> None:
        """Clean up scenario resources."""
        try:
            # Destroy all spawned actors
            if self.client and self.spawned_actors:
                batch = [carla.command.DestroyActor(x) for x in self.spawned_actors]
                self.client.apply_batch_sync(batch)
                self.spawned_actors.clear()
            print(f"Cleaned up {self.get_name()}")
        except Exception as e:
            print(f"Error during cleanup: {e}")
