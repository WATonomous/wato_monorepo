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
"""YAML-driven scenario for Software-In-the-Loop (SIL) component testing.

The scenario file is selected at runtime via the ``SIL_SCENARIO_FILE`` environment
variable, so this plugin loads through the existing scenario_server exactly like the
built-in scenarios (by module path ``carla_scenarios.scenarios.yaml_scenario``).

Coordinate convention: all poses in the YAML are expressed in the ROS map frame
(x forward, y left, z up, yaw in degrees CCW). They are converted to CARLA's
left-handed frame at spawn time so that the rest of the ROS stack (localization,
trajectory feeder, analyzer) operates in a single consistent frame. This is the
inverse of carla_common.carla_to_ros_position / carla_to_ros_rotation.
"""

import os
from typing import Optional

import yaml

from carla_scenarios.scenario_base import ScenarioBase

try:
    import carla
except ImportError:
    carla = None


def ros_pose_to_carla_transform(x: float, y: float, z: float, yaw_deg: float):
    """Convert a ROS-frame pose to a carla.Transform (inverse of carla_to_ros)."""
    return carla.Transform(
        carla.Location(x=float(x), y=float(-y), z=float(z)),
        carla.Rotation(yaw=float(-yaw_deg)),
    )


class YamlScenario(ScenarioBase):
    """Scenario plugin that builds a CARLA world from a YAML description."""

    def __init__(self):
        super().__init__()
        self.config: dict = {}
        self.config_path: Optional[str] = None

    def get_name(self) -> str:
        return self.config.get("name", "SIL YAML Scenario")

    def get_description(self) -> str:
        return f"SIL scenario loaded from {self.config_path or '<unset>'}"

    def initialize(self, client: "carla.Client") -> bool:
        if not self._load_config():
            return False
        return self._initialize_carla(client)

    def _load_config(self) -> bool:
        self.config_path = os.environ.get("SIL_SCENARIO_FILE")
        if not self.config_path:
            self._log("SIL_SCENARIO_FILE environment variable is not set", "error")
            return False
        if not os.path.isfile(self.config_path):
            self._log(f"Scenario file not found: {self.config_path}", "error")
            return False
        try:
            with open(self.config_path, "r") as f:
                self.config = yaml.safe_load(f) or {}
        except Exception as e:
            self._log(f"Failed to parse scenario YAML: {e}", "error")
            return False
        self._log(f"Loaded SIL scenario '{self.get_name()}' from {self.config_path}")
        return True

    def setup(self) -> bool:
        if self.world is None or carla is None:
            return False

        try:
            self._load_map(self.config.get("map", "Town10HD"))
            self._apply_weather(self.config.get("weather"))
            self._configure_determinism(self.config.get("seed"))

            if not self._spawn_ego(self.config.get("ego", {})):
                return False

            self._spawn_actors(self.config.get("actors", []) or [])

            self._log(f"SIL scenario setup complete: {self.get_name()}")
            return True
        except Exception as e:
            self._log(f"Failed to setup SIL scenario: {e}", "error")
            return False

    def _apply_weather(self, weather_name: Optional[str]) -> None:
        if not weather_name:
            return
        preset = getattr(carla.WeatherParameters, weather_name, None)
        if preset is None:
            self._log(f"Unknown weather preset '{weather_name}', skipping", "warn")
            return
        self.world.set_weather(preset)

    def _configure_determinism(self, seed: Optional[int]) -> None:
        """Pin the Traffic Manager seed so NPC behaviour is reproducible."""
        if seed is None:
            return
        try:
            tm = self.client.get_trafficmanager()
            tm.set_synchronous_mode(True)
            tm.set_random_device_seed(int(seed))
            self._log(f"Traffic Manager seeded with {seed} (synchronous)")
        except Exception as e:
            self._log(f"Could not configure Traffic Manager determinism: {e}", "warn")

    def _resolve_spawn_transform(self, spec: dict):
        """Build a carla.Transform from either an explicit ROS pose or a spawn index."""
        if "spawn_point_index" in spec:
            spawn_points = self.world.get_map().get_spawn_points()
            if not spawn_points:
                self._log("Map has no predefined spawn points", "error")
                return None
            idx = min(int(spec["spawn_point_index"]), len(spawn_points) - 1)
            return spawn_points[idx]

        tf = spec.get("transform")
        if not tf:
            self._log("Spawn spec missing 'transform' or 'spawn_point_index'", "error")
            return None
        return ros_pose_to_carla_transform(
            tf.get("x", 0.0), tf.get("y", 0.0), tf.get("z", 0.3), tf.get("yaw", 0.0)
        )

    def _spawn_ego(self, ego_spec: dict) -> bool:
        transform = self._resolve_spawn_transform(ego_spec)
        if transform is None:
            return False
        blueprint = ego_spec.get("blueprint", "vehicle.mini.cooper")
        ego = self.spawn_ego_vehicle(transform, blueprint_filter=blueprint)
        return ego is not None

    def _spawn_actors(self, actor_specs: list) -> None:
        blueprint_library = self.world.get_blueprint_library()
        for i, spec in enumerate(actor_specs):
            transform = self._resolve_spawn_transform(spec)
            if transform is None:
                self._log(f"Skipping actor {i}: invalid spawn", "warn")
                continue

            bp_filter = spec.get("blueprint", "vehicle.*")
            candidates = blueprint_library.filter(bp_filter)
            if not candidates:
                self._log(f"No blueprint matches '{bp_filter}' for actor {i}", "warn")
                continue
            blueprint = candidates[0]
            blueprint.set_attribute("role_name", spec.get("role_name", f"npc_{i}"))

            actor = self.world.try_spawn_actor(blueprint, transform)
            if actor is None:
                self._log(f"Failed to spawn actor {i} at {transform.location}", "warn")
                continue

            self._apply_actor_behavior(actor, spec.get("behavior", {}) or {})

    def _apply_actor_behavior(self, actor, behavior: dict) -> None:
        kind = behavior.get("kind", "static")
        try:
            if kind == "autopilot":
                actor.set_autopilot(True)
            elif kind == "constant_velocity":
                speed = float(behavior.get("speed_mps", 0.0))
                # CARLA applies this in the actor's local frame (x = forward).
                actor.enable_constant_velocity(carla.Vector3D(speed, 0.0, 0.0))
            elif kind == "static":
                actor.set_simulate_physics(True)
            else:
                self._log(f"Unknown actor behavior '{kind}', leaving static", "warn")
        except Exception as e:
            self._log(f"Failed to apply behavior '{kind}': {e}", "warn")
