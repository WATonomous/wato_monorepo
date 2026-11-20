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

import hashlib
import importlib
import json
import math
import random
import re
import threading
import xml.etree.ElementTree as ET

import yaml
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from typing import Optional, Dict
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from carla_msgs.srv import SwitchScenario, GetAvailableScenarios
from carla_msgs.msg import ScenarioStatus
from std_msgs.msg import Bool, Header
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
from rosgraph_msgs.msg import Clock

import carla
from carla_common import connect_carla
from carla_scenarios.scenario_base import ManifestScenario, ScenarioBase


class ScenarioServerNode(LifecycleNode):
    """Lifecycle node for managing CARLA scenarios."""

    def __init__(self, node_name="scenario_server"):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter(
            "carla_host",
            "localhost",
            ParameterDescriptor(description="CARLA server hostname"),
        )
        self.declare_parameter(
            "carla_port", 2000, ParameterDescriptor(description="CARLA server port")
        )
        self.declare_parameter(
            "carla_timeout",
            60.0,
            ParameterDescriptor(description="Connection timeout in seconds"),
        )
        self.declare_parameter(
            "initial_scenario",
            "carla_scenarios.scenarios.default_scenario",
            ParameterDescriptor(description="Scenario module path to load on startup"),
        )
        self.declare_parameter(
            "xodr_map_directory",
            "/opt/watonomous/maps/xodr",
            ParameterDescriptor(
                description="Directory containing OpenDRIVE maps used by scenarios"
            ),
        )
        # Simulation timing (see https://carla.readthedocs.io/en/latest/adv_synchrony_timestep/)
        self.declare_parameter(
            "carla_fps",
            60.0,
            ParameterDescriptor(
                description="Simulation frames per second (sets fixed_delta_seconds)"
            ),
        )
        self.declare_parameter(
            "synchronous_mode",
            False,
            ParameterDescriptor(
                description="Enable synchronous mode (server waits for client tick)"
            ),
        )
        self.declare_parameter(
            "no_rendering_mode",
            False,
            ParameterDescriptor(description="Disable rendering for faster simulation"),
        )
        self.declare_parameter(
            "substepping",
            True,
            ParameterDescriptor(description="Enable physics substepping"),
        )
        self.declare_parameter(
            "max_substep_delta_time",
            0.01,
            ParameterDescriptor(description="Max physics substep time in seconds"),
        )
        self.declare_parameter(
            "max_substeps",
            10,
            ParameterDescriptor(description="Max number of physics substeps per frame"),
        )

        self.declare_parameter("session_mode", False)

        # A single lock serializes setup, cleanup and ticks, including pause ACK.
        self.world_lock = threading.RLock()
        self.experiment_services = []
        self.control_enabled_publisher = None
        self.collision_publisher = None
        self.collision_sensor = None
        self.control_gate_client = None
        self.experiment_geometry = None
        self.collided = False

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.carla_world: Optional["carla.World"] = None
        self.current_scenario: Optional[ScenarioBase] = None
        self.current_scenario_name: str = ""
        self.available_scenarios: Dict[str, str] = {}

        # ROS interfaces (created in on_configure)
        self.status_publisher = None
        self.clock_publisher = None
        self.switch_scenario_service = None
        self.get_scenarios_service = None
        self.tick_timer = None
        self.paused = False
        self.pause_service = None
        self.prepare_switch_client = None
        self.client_cb_group = None
        self.service_cb_group = None

        self.get_logger().info(f"{node_name} initialized")

    def _apply_simulation_settings(self):
        """Apply simulation settings to CARLA world."""
        carla_fps = self.get_parameter("carla_fps").value
        sync_mode = self.get_parameter("synchronous_mode").value
        no_rendering = self.get_parameter("no_rendering_mode").value
        substepping = self.get_parameter("substepping").value
        max_substep_delta = self.get_parameter("max_substep_delta_time").value
        max_substeps = self.get_parameter("max_substeps").value

        if not math.isfinite(carla_fps) or carla_fps <= 0.0:
            raise ValueError("carla_fps must be finite and positive")
        if substepping and (max_substep_delta <= 0.0 or max_substeps <= 0):
            raise ValueError("Physics substep duration and count must be positive")
        if (
            sync_mode
            and substepping
            and 1.0 / carla_fps > max_substep_delta * max_substeps + 1e-9
        ):
            raise ValueError("fixed_delta_seconds exceeds the physics substep budget")

        # Sync mode: fixed timestep, server waits for tick()
        # Async mode: variable timestep
        if sync_mode:
            fixed_delta = 1.0 / carla_fps
        else:
            fixed_delta = 0.0

        # Create explicit settings rather than modifying existing
        # This ensures we don't inherit unexpected values after load_world()
        settings = carla.WorldSettings(
            synchronous_mode=sync_mode,
            no_rendering_mode=no_rendering,
            fixed_delta_seconds=fixed_delta,
            substepping=substepping,
            max_substep_delta_time=max_substep_delta,
            max_substeps=max_substeps,
        )
        self.carla_world.apply_settings(settings)

        # Verify settings were applied
        if sync_mode:
            self.get_logger().info(
                f"Applied settings: sync=True, fixed_delta={fixed_delta:.6f} ({carla_fps} FPS), "
                f"no_rendering={no_rendering}, substepping={substepping}"
            )
        else:
            self.get_logger().info(
                f"Applied settings: sync=False, variable_timestep, "
                f"no_rendering={no_rendering}, substepping={substepping}"
            )

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info("Configuring...")

        # Get parameters
        host = self.get_parameter("carla_host").value
        port = self.get_parameter("carla_port").value
        timeout = self.get_parameter("carla_timeout").value

        # Connect to CARLA
        try:
            self.carla_client = connect_carla(host, port, timeout)
            server_version = self.carla_client.get_server_version()
            client_version = self.carla_client.get_client_version()
            if (
                self.get_parameter("session_mode").value
                and server_version != client_version
            ):
                raise ValueError(
                    f"CARLA client {client_version} != server {server_version}"
                )
            self.get_logger().info(
                f"Connected to CARLA at {host}:{port}, server={server_version}, client={client_version}"
            )

            # Get world and configure simulation mode
            self.carla_world = self.carla_client.get_world()
            self._apply_simulation_settings()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return TransitionCallbackReturn.FAILURE

        # Create ROS interfaces
        self.status_publisher = self.create_lifecycle_publisher(
            ScenarioStatus, "~/scenario_status", 10
        )

        # Clock publisher for simulation time (not lifecycle - always active)
        self.clock_publisher = self.create_publisher(Clock, "/clock", 10)

        # Separate callback group for services to prevent blocking by tick timer
        self.service_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()

        self.switch_scenario_service = self.create_service(
            SwitchScenario,
            "~/switch_scenario",
            self.switch_scenario_callback,
            callback_group=self.service_cb_group,
        )

        self.get_scenarios_service = self.create_service(
            GetAvailableScenarios,
            "~/get_available_scenarios",
            self.get_scenarios_callback,
            callback_group=self.service_cb_group,
        )

        self.pause_service = self.create_service(
            SetBool,
            "pause",
            self.pause_callback,
            callback_group=self.service_cb_group,
        )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.control_enabled_publisher = self.create_publisher(
            Bool, "/carla/control_enabled", qos
        )
        self.collision_publisher = self.create_publisher(
            Bool, "/carla/experiment_collision", qos
        )
        self._publish_control_enabled(not self.get_parameter("session_mode").value)
        self.experiment_services = [
            self.create_service(
                SwitchScenario,
                "~/prepare_experiment",
                self.prepare_experiment_callback,
                callback_group=self.service_cb_group,
            ),
            self.create_service(
                Trigger,
                "~/hold",
                self.hold_callback,
                callback_group=self.service_cb_group,
            ),
            self.create_service(
                Trigger,
                "~/release",
                self.release_callback,
                callback_group=self.service_cb_group,
            ),
            self.create_service(
                Trigger,
                "~/close_experiment",
                self.close_experiment_callback,
                callback_group=self.service_cb_group,
            ),
        ]

        # Create client for lifecycle manager's prepare_for_scenario_switch service
        # Uses namespace-relative path - both nodes share the same namespace
        self.client_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.prepare_switch_client = self.create_client(
            Trigger,
            "prepare_for_scenario_switch",
            callback_group=self.client_cb_group,
        )

        self.control_gate_client = self.create_client(
            SetBool,
            "/carla/ackermann_control/set_enabled",
            callback_group=self.client_cb_group,
        )

        # Discover available scenarios
        self._discover_scenarios()

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        # Plugins request controlled setup ticks through this server. Start the
        # recurring timer only after setup succeeds, avoiding orphaned timers
        # when lifecycle activation is retried.
        sync_mode = self.get_parameter("synchronous_mode").value
        carla_fps = self.get_parameter("carla_fps").value
        timer_period = (1.0 / carla_fps) if sync_mode else 0.001
        self.paused = self.get_parameter("session_mode").value

        # Load initial scenario
        initial_scenario = (
            ""
            if self.get_parameter("session_mode").value
            else self.get_parameter("initial_scenario").value
        )
        if initial_scenario:
            success = self._load_scenario(initial_scenario)
            if not success:
                self.get_logger().error(
                    f"Failed to load initial scenario: {initial_scenario}"
                )
                return TransitionCallbackReturn.FAILURE

        self.tick_timer = self.create_timer(timer_period, self._tick_callback)
        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info("Deactivating...")

        # Stop tick timer
        if self.tick_timer:
            self.destroy_timer(self.tick_timer)
            self.tick_timer = None

        # Unload current scenario
        self._unload_scenario()

        self.get_logger().info("Deactivation complete")
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info("Cleaning up...")

        # Destroy ROS interfaces
        if self.switch_scenario_service:
            self.destroy_service(self.switch_scenario_service)
            self.switch_scenario_service = None

        if self.get_scenarios_service:
            self.destroy_service(self.get_scenarios_service)
            self.get_scenarios_service = None

        if self.status_publisher:
            self.destroy_publisher(self.status_publisher)
            self.status_publisher = None

        if self.clock_publisher:
            self.destroy_publisher(self.clock_publisher)
            self.clock_publisher = None

        if self.prepare_switch_client:
            self.destroy_client(self.prepare_switch_client)
            self.prepare_switch_client = None

        if self.pause_service:
            self.destroy_service(self.pause_service)
            self.pause_service = None
        for service in self.experiment_services:
            self.destroy_service(service)
        self.experiment_services.clear()
        for publisher in (self.control_enabled_publisher, self.collision_publisher):
            if publisher:
                self.destroy_publisher(publisher)
        self.control_enabled_publisher = None
        self.collision_publisher = None

        if self.control_gate_client:
            self.destroy_client(self.control_gate_client)
            self.control_gate_client = None

        # Disconnect from CARLA
        self.carla_client = None

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def _tick_callback(self):
        """Serialize ticks so a successful pause acknowledges a settled boundary."""
        with self.world_lock:
            self._tick_locked()

    def _advance_world(self):
        """Only this server advances CARLA, including controlled plugin setup ticks."""
        with self.world_lock:
            self.carla_world = self.carla_client.get_world()
            if self.get_parameter("synchronous_mode").value:
                self.carla_world.tick()
            else:
                self.carla_world.wait_for_tick()
            if self.clock_publisher:
                sim_time = self.carla_world.get_snapshot().timestamp.elapsed_seconds
                clock_msg = Clock()
                clock_msg.clock.sec = int(sim_time)
                clock_msg.clock.nanosec = int((sim_time % 1.0) * 1e9)
                self.clock_publisher.publish(clock_msg)

    def _tick_locked(self):
        self._publish_scenario_status()
        if self.current_scenario and self.collision_publisher:
            self.collision_publisher.publish(Bool(data=self.collided))
        if self.paused:
            return

        if self.carla_world:
            try:
                self._advance_world()
            except Exception as e:
                self.get_logger().warn(f"Error waiting for tick: {e}")

        # Execute scenario logic
        if self.current_scenario:
            try:
                self.current_scenario.execute()
            except Exception as e:
                self.get_logger().error(f"Error executing scenario: {e}")

        self._publish_scenario_status()

    def _publish_scenario_status(self):
        # Publish status
        if self.status_publisher and self.status_publisher.is_activated:
            status_msg = ScenarioStatus()
            status_msg.header = Header()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.scenario_name = self.current_scenario_name
            status_msg.description = (
                self.current_scenario.get_description() if self.current_scenario else ""
            )
            status_msg.state = "running" if self.current_scenario else "idle"
            status_msg.info = ""
            self.status_publisher.publish(status_msg)

    def pause_callback(self, request, response):
        """Handle pause service request. data=True pauses, data=False resumes."""
        try:
            with self.world_lock:
                if not request.data and self.get_parameter("session_mode").value:
                    if not self.current_scenario:
                        response.success = False
                        response.message = (
                            "No experiment loaded; run watod sim open first."
                        )
                        return response
                self.paused = request.data
                held = (
                    isinstance(self.current_scenario, ManifestScenario)
                    and self.current_scenario.held
                )
                self._publish_control_enabled(not self.paused and not held)
                response.success = True
                response.message = (
                    "paused at tick boundary" if self.paused else "resumed"
                )
                return response
        except Exception as error:
            response.success = False
            response.message = str(error)
            return response

    def _publish_control_enabled(self, enabled):
        # The acknowledged service fences queued commands before a tick can
        # resume; the latched topic also informs components joining later.
        if self.control_gate_client and self.control_gate_client.service_is_ready():
            future = self.control_gate_client.call_async(SetBool.Request(data=enabled))
            finished = threading.Event()
            future.add_done_callback(lambda _: finished.set())
            if not finished.wait(5.0):
                self.control_gate_client.remove_pending_request(future)
                raise RuntimeError(
                    "/carla/ackermann_control/set_enabled exceeded 5s; run watod sim close"
                )
            result = future.result()
            if result is None or not result.success:
                raise RuntimeError("CARLA command fence rejected the transition")
        if self.control_enabled_publisher:
            self.control_enabled_publisher.publish(Bool(data=enabled))

    def _cleanup_bridge(self):
        """Wait boundedly for old actor handles to be released before world reload."""
        if not self.current_scenario:
            return
        timeout = float(self.get_parameter("carla_timeout").value)
        if not self.prepare_switch_client.wait_for_service(
            timeout_sec=min(timeout, 10.0)
        ):
            raise RuntimeError(
                "Missing /carla/prepare_for_scenario_switch; run watod sim stop then setup"
            )
        future = self.prepare_switch_client.call_async(Trigger.Request())
        finished = threading.Event()
        future.add_done_callback(lambda _: finished.set())
        if not finished.wait(timeout):
            self.prepare_switch_client.remove_pending_request(future)
            raise RuntimeError(
                f"/carla/prepare_for_scenario_switch exceeded {timeout}s"
            )
        result = future.result()
        if result is None or not result.success:
            raise RuntimeError("Bridge cleanup failed before world switch")

    def _read_playground(self, name):
        """Preflight definition, map XML, blueprint and geometry before mutation."""
        if re.fullmatch(r"[a-z][a-z0-9_]*", name) is None:
            raise ValueError(f"Invalid playground name: {name}")
        path = (
            Path(get_package_share_directory("carla_scenarios"))
            / "config"
            / "playgrounds"
            / f"{name}.yaml"
        )
        if not path.is_file():
            raise ValueError(f"Missing playground definition: {path}")
        definition = yaml.safe_load(path.read_text())
        if (
            not isinstance(definition, dict)
            or definition.get("version") != 1
            or definition.get("id") != name
        ):
            raise ValueError(f"Invalid version/id in {path}")
        seed = definition.get("seed", 42)
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError(f"seed must be a nonnegative integer in {path}")
        spawn = definition.get("spawn", {})
        if not isinstance(spawn, dict):
            raise ValueError(f"spawn must be a YAML mapping in {path}")
        for key in ("s_margin_m", "z_offset_m"):
            value = float(spawn.get(key, 10.0 if key == "s_margin_m" else 0.4))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(
                    f"spawn.{key} must be finite and nonnegative in {path}"
                )
        map_name = definition.get("map", {}).get("xodr")
        if (
            not isinstance(map_name, str)
            or Path(map_name).name != map_name
            or not map_name.endswith(".xodr")
        ):
            raise ValueError(f"Expected map.xodr filename in {path}")
        map_path = Path(self.get_parameter("xodr_map_directory").value) / map_name
        try:
            source = map_path.read_text(encoding="utf-8")
        except OSError as error:
            raise ValueError(
                f"Map unreadable: {map_path}; provide the converted map and retry open"
            ) from error
        document = ET.fromstring(source)
        if document.tag != "OpenDRIVE" or not document.findall("road"):
            raise ValueError(f"Invalid OpenDRIVE document: {map_path}")
        for key in ("required_straight_m", "goal_distance_m"):
            value = float(definition.get(key, 0.0))
            if not math.isfinite(value) or value < 0.0:
                raise ValueError(f"{key} must be finite and nonnegative in {path}")
        if definition.get("parked"):
            parked_distance = float(definition["parked"]["distance_m"])
            if not math.isfinite(parked_distance) or parked_distance <= 0.0:
                raise ValueError(
                    f"parked.distance_m must be finite and positive in {path}"
                )
        self.carla_world = self.carla_client.get_world()
        library = self.carla_world.get_blueprint_library()
        blueprints = [definition.get("ego_blueprint", "vehicle.mini.cooper")]
        if definition.get("parked"):
            blueprints.append(
                definition["parked"].get("blueprint", "vehicle.mini.cooper")
            )
        for blueprint in blueprints:
            if not library.filter(blueprint):
                raise ValueError(f"CARLA blueprint unavailable: {blueprint}")
        metadata_path = Path(str(map_path) + ".metadata.json")
        provenance = None
        if metadata_path.is_file():
            provenance = json.loads(metadata_path.read_text())
            actual_digest = hashlib.sha256(map_path.read_bytes()).hexdigest()
            source_path = map_path.parent.parent / "osm" / (map_path.stem + ".osm")
            if provenance.get("xodr_sha256") != actual_digest:
                raise ValueError(
                    f"Converted map digest mismatch: {metadata_path}; reconvert {source_path}"
                )
            if not source_path.is_file():
                raise ValueError(f"Missing Lanelet2 source map: {source_path}")
            if (
                provenance.get("source_sha256")
                != hashlib.sha256(source_path.read_bytes()).hexdigest()
            ):
                raise ValueError(
                    f"Lanelet2/OpenDRIVE map versions differ: {source_path}; reconvert the map"
                )
        if definition.get("require_map_provenance", False) and provenance is None:
            raise ValueError(
                f"Missing converted-map provenance: {metadata_path}; reconvert the matching "
                "Lanelet2 map before opening this playground"
            )
        scenario = ManifestScenario(definition, map_path.resolve())
        scenario.provenance = provenance
        scenario.logger = self.get_logger()
        scenario.xodr_map_directory = map_path.parent
        # Client-side OpenDRIVE parsing proves the selected continuous stretch
        # without loading a new CARLA world or disturbing the old experiment.
        waypoint = scenario._spawn_waypoint(carla.Map(name, source))
        distances = [float(definition.get("goal_distance_m", 0.0))]
        if definition.get("parked"):
            distances.append(float(definition["parked"]["distance_m"]))
        for distance in distances:
            if distance > 0.0 and len(waypoint.next(distance)) != 1:
                raise ValueError(
                    f"Actor or goal at {distance} m leaves the continuous lane in {path}"
                )
        return scenario

    def prepare_experiment_callback(self, request, response):
        """Load a reviewed playground while holding the ego for ROS readiness."""
        response.previous_scenario = self.current_scenario_name
        try:
            with self.world_lock:
                scenario = self._read_playground(request.scenario_name)
            self._cleanup_bridge()
            with self.world_lock:
                self._publish_control_enabled(False)
                self.paused = True
                self._unload_scenario()
                self.current_scenario_name = ""
                self._publish_scenario_status()
                seed = int(scenario.definition.get("seed", 42))
                random.seed(seed)
                scenario.advance_world = self._advance_world
                scenario.initialize(self.carla_client)
                # Register ownership before setup so partial failures are cleanable.
                self.current_scenario = scenario
                if not scenario.setup():
                    raise RuntimeError("World setup failed")
                self.carla_world = self.carla_client.get_world()
                self._apply_simulation_settings()
                self.current_scenario_name = request.scenario_name
                sensor_bp = self.carla_world.get_blueprint_library().find(
                    "sensor.other.collision"
                )
                self.collision_sensor = self.carla_world.spawn_actor(
                    sensor_bp, carla.Transform(), attach_to=scenario.ego_vehicle
                )
                scenario.owned_actors.append(self.collision_sensor)
                self.collided = False
                self.collision_publisher.publish(Bool(data=False))
                sensor_id = self.collision_sensor.id
                self.collision_sensor.listen(
                    lambda event: self._record_collision(sensor_id)
                )
                settings = self.carla_world.get_settings()
                scenario.geometry["settings"] = {
                    "synchronous_mode": settings.synchronous_mode,
                    "fixed_delta_seconds": settings.fixed_delta_seconds,
                    "substepping": settings.substepping,
                    "max_substep_delta_time": settings.max_substep_delta_time,
                    "max_substeps": settings.max_substeps,
                }
                scenario.geometry["map"]["provenance"] = scenario.provenance
                scenario.geometry["map_source_sha256"] = (
                    scenario.provenance.get("source_sha256")
                    if scenario.provenance
                    else None
                )
                scenario.geometry["server_version"] = (
                    self.carla_client.get_server_version()
                )
                scenario.geometry["client_version"] = (
                    self.carla_client.get_client_version()
                )
                scenario.geometry["sim_time_s"] = (
                    self.carla_world.get_snapshot().timestamp.elapsed_seconds
                )
                self.experiment_geometry = scenario.geometry
                # Ego physics is held; ticks may now populate TF, odometry/sensors.
                self.paused = False
                response.success = True
                response.message = json.dumps(self.experiment_geometry)
        except Exception as error:
            self.get_logger().error(f"Experiment preparation failed: {error}")
            response.success = False
            response.message = str(error)
            # A preflight failure has no side effects. A setup failure owns only
            # the partially prepared plugin, which remains closeable for recovery.
        return response

    def _record_collision(self, sensor_id):
        # CARLA sensor callbacks may arrive after a world switch. Ignore an old
        # sensor's queued event rather than attributing it to the next attempt.
        if self.collision_sensor and self.collision_sensor.id == sensor_id:
            self.collided = True

    def hold_callback(self, request, response):
        try:
            with self.world_lock:
                if not isinstance(self.current_scenario, ManifestScenario):
                    response.success = False
                    response.message = "No prepared experiment"
                    return response
                self._publish_control_enabled(False)
                self.current_scenario.set_held(True)
                response.success = True
                response.message = "ego held stationary"
                return response
        except Exception as error:
            response.success = False
            response.message = str(error)
            return response

    def release_callback(self, request, response):
        try:
            with self.world_lock:
                if not isinstance(self.current_scenario, ManifestScenario):
                    response.success = False
                    response.message = "No prepared experiment"
                    return response
                self.current_scenario.set_held(False)
                self._publish_control_enabled(not self.paused)
                response.success = True
                response.message = "ego released"
                return response
        except Exception as error:
            response.success = False
            response.message = str(error)
            return response

    def close_experiment_callback(self, request, response):
        try:
            self._cleanup_bridge()
            with self.world_lock:
                self._publish_control_enabled(False)
                self.paused = True
                self._unload_scenario()
                self.current_scenario_name = ""
                self.experiment_geometry = None
                self._publish_scenario_status()
            response.success = True
            response.message = "experiment released; session remains available"
        except Exception as error:
            response.success = False
            response.message = str(error)
        return response

    def switch_scenario_callback(self, request, response):
        """Handle switch scenario service request."""
        self.get_logger().info(
            f"Received switch_scenario request: {request.scenario_name}"
        )

        previous = self.current_scenario_name

        # Request lifecycle manager to cleanup managed nodes first
        if self.prepare_switch_client and self.current_scenario:
            if self.prepare_switch_client.service_is_ready():
                self.get_logger().info(
                    "Requesting lifecycle manager to cleanup nodes..."
                )
                try:
                    result = self.prepare_switch_client.call(Trigger.Request())
                    if result.success:
                        self.get_logger().info("Lifecycle manager cleanup complete")
                    else:
                        self.get_logger().warn(
                            f"Lifecycle manager cleanup failed: {result.message}"
                        )
                except Exception as e:
                    self.get_logger().warn(f"Error calling lifecycle manager: {e}")
            else:
                self.get_logger().warn(
                    "Lifecycle manager service not ready, proceeding anyway"
                )
        else:
            self.get_logger().info("No current scenario, skipping lifecycle cleanup")

        self.get_logger().info("Proceeding with scenario load...")

        success = self._load_scenario(request.scenario_name)

        response.success = success
        response.previous_scenario = previous
        response.message = (
            f"Switched to {request.scenario_name}"
            if success
            else "Failed to switch scenario"
        )

        return response

    def get_scenarios_callback(self, request, response):
        """Handle get available scenarios service request."""
        response.scenario_names = list(self.available_scenarios.keys())
        response.descriptions = list(self.available_scenarios.values())
        return response

    def _discover_scenarios(self):
        """Discover available scenarios."""
        builtin_scenarios = {
            "carla_scenarios.scenarios.default_scenario": "Default Ego Spawn",
            "carla_scenarios.scenarios.empty_scenario": "Empty World (no NPCs)",
            "carla_scenarios.scenarios.light_traffic_scenario": "Light Traffic",
            "carla_scenarios.scenarios.heavy_traffic_scenario": "Heavy Traffic",
            "carla_scenarios.scenarios.custom_scenario": "Custom Scenario",
            "carla_scenarios.scenarios.oval_scenario": "Oval OpenDRIVE Map",
            "carla_scenarios.scenarios.ring_road_scenario": "Ring Road OpenDRIVE Map",
        }
        self.available_scenarios.update(builtin_scenarios)

        self.get_logger().info(f"Discovered {len(self.available_scenarios)} scenarios")

    def _unload_scenario(self):
        """Release plugin-owned actors; never sweep unrelated CARLA actors."""
        with self.world_lock:
            if self.current_scenario:
                self.current_scenario.cleanup()
            self.current_scenario = None
            self.collision_sensor = None

    def _load_scenario(self, scenario_module_path: str) -> bool:
        """Serialize manual plugin setup with the world's tick authority."""
        with self.world_lock:
            return self._load_scenario_locked(scenario_module_path)

    def _load_scenario_locked(self, scenario_module_path: str) -> bool:
        """Load and initialize a scenario."""
        # Unload any existing scenario first
        self._unload_scenario()

        try:
            # Import scenario module
            # scenario_module_path: e.g. 'carla_scenarios.scenarios.default_scenario'
            # class_name derived from last part: 'default_scenario' -> 'DefaultScenario'
            parts = scenario_module_path.rsplit(".", 1)
            if len(parts) != 2:
                self.get_logger().error(
                    f"Invalid scenario path: {scenario_module_path}"
                )
                return False

            class_name = parts[1]
            # Convert module name to class name (e.g., default_scenario -> DefaultScenario)
            if not class_name[0].isupper():
                class_name = "".join(
                    word.capitalize() for word in class_name.split("_")
                )

            module = importlib.import_module(scenario_module_path)
            scenario_class = getattr(module, class_name)

            # Instantiate and initialize scenario
            scenario = scenario_class()
            scenario.advance_world = self._advance_world
            scenario.logger = self.get_logger()
            scenario.xodr_map_directory = Path(
                self.get_parameter("xodr_map_directory").value
            )
            if not scenario.initialize(self.carla_client):
                self.get_logger().error(
                    f"Failed to initialize scenario: {scenario_module_path}"
                )
                return False

            self.current_scenario = scenario
            if not scenario.setup():
                self.get_logger().error(
                    f"Failed to setup scenario: {scenario_module_path}"
                )
                return False

            # Refresh world reference and reapply settings (load_world resets to defaults)
            self.carla_world = self.carla_client.get_world()
            self._apply_simulation_settings()

            self.current_scenario = scenario
            self.current_scenario_name = scenario_module_path
            self.get_logger().info(f"Loaded scenario: {scenario.get_name()}")
            return True

        except Exception as e:
            self.get_logger().error(
                f"Error loading scenario {scenario_module_path}: {e}"
            )
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioServerNode()

    # Use MultiThreadedExecutor to allow service calls from within callbacks
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=5)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
