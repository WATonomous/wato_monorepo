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

import importlib
import math
from pathlib import Path
import threading
import time

from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from typing import Optional, Dict
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rcl_interfaces.msg import ParameterDescriptor
from carla_msgs.srv import SwitchScenario, GetAvailableScenarios
from carla_msgs.msg import ScenarioStatus
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

import carla
from carla_common import connect_carla
from carla_scenarios.scenario_base import ScenarioBase
from carla_scenarios.map_registry import Registry
from carla_scenarios.world_model_reload import WorldModelReload, call


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
            10.0,
            ParameterDescriptor(description="Connection timeout in seconds"),
        )
        self.declare_parameter(
            "initial_scenario",
            "carla_scenarios.scenarios.default_scenario",
            ParameterDescriptor(description="Scenario module path to load on startup"),
        )
        self.declare_parameter("scenario_registry", str(
            Path(get_package_share_directory("carla_scenarios")) / "config/scenarios.yaml"))
        self.declare_parameter("maps_root", "/opt/watonomous/maps")
        self.declare_parameter("world_model_node", "/world_modeling/world_model")
        self.registry = Registry(self.get_parameter("scenario_registry").value,
                                 self.get_parameter("maps_root").value)
        self.operation_lock = threading.Lock()
        self.world_lock = threading.RLock()
        self.state = "idle"
        self.info = ""
        self.generation = 0
        self.bundle = None
        self.specification = None
        self.ego_spawn = None
        self.odom_counter = 0
        self.latest_odom = None
        self.idle_counter = 0
        self.controller_idle = False
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

        if not math.isfinite(carla_fps) or carla_fps <= 0:
            raise ValueError("carla_fps must be finite and positive")
        if not math.isfinite(max_substep_delta) or max_substep_delta <= 0 or max_substeps < 1:
            raise ValueError("physics substep size/count must be positive")
        if sync_mode and substepping and 1.0 / carla_fps > max_substep_delta * max_substeps:
            raise ValueError("fixed timestep exceeds max_substep_delta_time * max_substeps")

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
            self.get_logger().info(f"Connected to CARLA at {host}:{port}")

            # Get world and configure simulation mode
            self.carla_world = self.carla_client.get_world()
            self._apply_simulation_settings()
        except Exception as e:
            self.get_logger().error(f"Failed to connect to CARLA: {e}")
            return TransitionCallbackReturn.FAILURE

        # Create ROS interfaces
        retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                              reliability=ReliabilityPolicy.RELIABLE)
        self.status_publisher = self.create_publisher(ScenarioStatus, "~/scenario_status", retained)

        # Clock publisher for simulation time (not lifecycle - always active)
        self.clock_publisher = self.create_publisher(Clock, "/clock", 10)

        # Separate callback group for services to prevent blocking by tick timer
        self.service_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.tick_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.odom_subscription = self.create_subscription(
            Odometry, "/ego/odom", self.odom_callback, 10, callback_group=self.service_cb_group)
        self.idle_subscription = self.create_subscription(
            Bool, "/action/is_idle", self.idle_callback, 10, callback_group=self.service_cb_group)

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

        # Create client for lifecycle manager's prepare_for_scenario_switch service
        # Uses namespace-relative path - both nodes share the same namespace
        self.client_cb_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.prepare_switch_client = self.create_client(
            Trigger,
            "prepare_for_scenario_switch",
            callback_group=self.client_cb_group,
        )

        self.finish_switch_client = self.create_client(
            Trigger, "finish_scenario_switch", callback_group=self.client_cb_group)
        self.stop_injection_client = self.create_client(
            Trigger, "/carla/carla_fake_planner/stop", callback_group=self.client_cb_group)
        self.reset_service = self.create_service(
            Trigger, "~/reset_ego", self.reset_callback, callback_group=self.service_cb_group)
        self.world_model = WorldModelReload(self, self.get_parameter("world_model_node").value,
                                           self.client_cb_group)
        self.status_timer = self.create_timer(0.5, self.publish_status,
                                              callback_group=self.service_cb_group)
        self.reconcile_timer = self.create_timer(2.0, self.reconcile_world_model,
                                                 callback_group=self.service_cb_group)

        # Discover available scenarios
        self._discover_scenarios()

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        # Create tick timer before loading scenario (sync mode needs ticks for spawning)
        sync_mode = self.get_parameter("synchronous_mode").value
        carla_fps = self.get_parameter("carla_fps").value
        timer_period = (1.0 / carla_fps) if sync_mode else 0.001
        self.tick_timer = self.create_timer(timer_period, self._tick_callback,
                                             callback_group=self.tick_cb_group)

        # Load initial scenario
        initial_scenario = self.get_parameter("initial_scenario").value
        if initial_scenario:
            success = self.switch(initial_scenario, initial=True)
            if not success:
                self.get_logger().error(
                    f"Failed to load initial scenario: {initial_scenario}"
                )
                self.destroy_timer(self.tick_timer)
                self.tick_timer = None
                return TransitionCallbackReturn.FAILURE

        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        if not self.operation_lock.acquire(blocking=False):
            return TransitionCallbackReturn.FAILURE
        try:
            self._stop_injection()
            self.state, self.paused = "idle", True
            self.publish_status()
            result = call(self.prepare_switch_client, Trigger.Request(), timeout=90.0)
            if not result.success:
                raise RuntimeError(result.message)
            if self.tick_timer:
                self.destroy_timer(self.tick_timer)
                self.tick_timer = None
            with self.world_lock:
                self._stop_vehicle()
                self._unload_scenario()
            return super().on_deactivate(state)
        except Exception as error:
            self.get_logger().error(str(error))
            return TransitionCallbackReturn.FAILURE
        finally:
            self.operation_lock.release()

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        # Lifecycle reconfiguration must not leave duplicate callbacks/services.
        self.state, self.paused = "idle", True
        for name in ("tick_timer", "status_timer", "reconcile_timer"):
            value = getattr(self, name, None)
            if value:
                self.destroy_timer(value)
                setattr(self, name, None)
        for kind, names in (
            ("service", ("switch_scenario_service", "get_scenarios_service", "pause_service", "reset_service")),
            ("subscription", ("odom_subscription", "idle_subscription")),
            ("publisher", ("status_publisher", "clock_publisher")),
            ("client", ("prepare_switch_client", "finish_switch_client", "stop_injection_client")),
        ):
            for name in names:
                value = getattr(self, name, None)
                if value:
                    getattr(self, "destroy_" + kind)(value)
                    setattr(self, name, None)
        if getattr(self, "world_model", None):
            for client in self.world_model.clients:
                self.destroy_client(client)
            self.world_model = None
        with self.world_lock:
            self._unload_scenario()
            self.carla_client, self.carla_world = None, None
        self.bundle, self.specification, self.ego_spawn = None, None, None
        self.latest_odom = None
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.state, self.paused = "idle", True
        with self.world_lock:
            self._stop_vehicle()
        return self.on_cleanup(state)

    def _tick_callback(self):
        # Loading mutates world/actor references under the same lock. During
        # starting, ticks are permitted so bridge configure can wait_for_tick.
        if self.paused or self.state not in ("starting", "running"):
            return
        with self.world_lock:
            if self.carla_world is None:
                return
            try:
                if self.get_parameter("synchronous_mode").value:
                    self.carla_world.tick()
                else:
                    self.carla_world.wait_for_tick()
                snapshot = self.carla_world.get_snapshot()
                sim_time = snapshot.timestamp.elapsed_seconds
                clock_msg = Clock()
                clock_msg.clock.sec = int(sim_time)
                clock_msg.clock.nanosec = int((sim_time % 1.0) * 1e9)
                self.clock_publisher.publish(clock_msg)
                if self.current_scenario:
                    self.current_scenario.execute()
            except Exception as error:
                self.get_logger().error(f"Simulation tick failed: {error}")

    def publish_status(self):
        if self.status_publisher is None:
            return
        message = ScenarioStatus()
        message.header.stamp = self.get_clock().now().to_msg()
        message.scenario_name = self.current_scenario_name
        message.description = self.current_scenario.get_description() if self.current_scenario else ""
        message.state = "paused" if self.paused and self.state == "running" else self.state
        message.info = self.info
        message.generation = self.generation
        if self.bundle:
            message.map_id = self.bundle.id
            message.osm_map_path = str(self.bundle.osm_path)
            message.projector_type = self.bundle.lanelet.get("projector", "local_cartesian")
            message.origin_lat = float(self.bundle.origin["lat"])
            message.origin_lon = float(self.bundle.origin["lon"])
            message.trajectories_enabled = bool(self.specification.get("trajectories_enabled", False))
        self.status_publisher.publish(message)

    def _stop_vehicle(self):
        if self.carla_world:
            actors = self.carla_world.get_actors().filter("vehicle.*")
            for ego in actors:
                if ego.attributes.get("role_name") == "ego_vehicle":
                    ego.set_autopilot(False)
                    ego.apply_control(carla.VehicleControl(brake=1.0))
                    return ego
        return None

    def idle_callback(self, message):
        self.controller_idle = message.data
        self.idle_counter += 1

    def _stop_injection(self):
        after = self.idle_counter
        result = call(self.stop_injection_client, Trigger.Request())
        if not result.success:
            raise RuntimeError(result.message)
        if self.count_publishers("/action/is_idle"):
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                if self.idle_counter > after and self.controller_idle:
                    return
                time.sleep(0.02)
            raise RuntimeError("action controller did not acknowledge cleared trajectory on /action/is_idle")

    def odom_callback(self, message):
        self.latest_odom = message
        self.odom_counter += 1

    def wait_localization(self, pose, after):
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if self.odom_counter > after and self.latest_odom is not None:
                position = self.latest_odom.pose.pose.position
                if math.hypot(position.x - pose[0], position.y - pose[1]) < 0.2:
                    return
            time.sleep(0.02)
        raise RuntimeError("fresh ego localization was not published after load/reset")

    def pause_callback(self, request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success, response.message = False, "Scenario operation in progress"
            return response
        try:
            if self.state != "running":
                raise ValueError("simulation is not ready")
            self.paused = request.data
            if self.paused:
                with self.world_lock:
                    self._stop_vehicle()
            self.publish_status()
            response.success, response.message = True, "paused" if self.paused else "resumed"
        except Exception as error:
            response.success, response.message = False, str(error)
        finally:
            self.operation_lock.release()
        return response

    def switch(self, name, initial=False):
        if not self.operation_lock.acquire(blocking=False):
            self.info = "Scenario operation in progress"
            return False
        destructive = False
        try:
            # All file, converter and plugin errors are checked before teardown.
            scenario_id, specification, bundle = self.registry.scenario(name)
            bundle.prepare()
            built_in = bundle.data["carla"].get("built_in_map")
            if built_in and not any(path.rsplit("/", 1)[-1] == built_in
                                    for path in self.carla_client.get_available_maps()):
                raise ValueError(f"CARLA does not have built-in map {built_in}")
            module = importlib.import_module(specification["module"])
            class_name = "".join(word.capitalize() for word in specification["module"].rsplit(".", 1)[1].split("_"))
            scenario = getattr(module, class_name)()
            scenario.map_bundle = bundle
            scenario.logger = self.get_logger()
            self._stop_injection()
            destructive = True
            self.state, self.info = "loading", "Stopping bridge and map consumers"
            self.publish_status()
            self.paused = True
            with self.world_lock:
                self._stop_vehicle()
            if not initial or self.generation > 0:
                result = call(self.prepare_switch_client, Trigger.Request(), timeout=90.0)
                if not result.success:
                    raise RuntimeError(result.message)
            self.world_model.prepare()
            old_odom_counter = self.odom_counter
            with self.world_lock:
                self._unload_scenario()
                if not scenario.initialize(self.carla_client) or not scenario.setup():
                    raise RuntimeError(f"scenario setup failed: {scenario_id}")
                self.carla_world = self.carla_client.get_world()
                self._apply_simulation_settings()
                ego = self._stop_vehicle()
                if ego is None:
                    raise RuntimeError("scenario did not spawn an ego_vehicle")
                self.ego_spawn = ego.get_transform()
                self.current_scenario = scenario
                self.current_scenario_name = scenario_id
                self.bundle, self.specification = bundle, specification
                self.generation += 1
                self.state, self.info = "starting", "Rebinding bridge and loading Lanelet2"
                self.paused = False
            self.publish_status()
            result = call(self.finish_switch_client, Trigger.Request(), timeout=90.0)
            if not result.success:
                raise RuntimeError(result.message)
            transform = self.ego_spawn
            pose = (transform.location.x, -transform.location.y, -math.radians(transform.rotation.yaw))
            self.wait_localization(pose, old_odom_counter)
            self.world_model.apply(bundle, pose, self.generation)
            self.state, self.info = "running", ""
            self.publish_status()
            return True
        except Exception as error:
            self.info = str(error)
            self.get_logger().error(self.info)
            if destructive:
                self.paused = True
                self.state = "error"
                try:
                    with self.world_lock:
                        self.carla_world = self.carla_client.get_world()
                        self._stop_vehicle()
                except Exception:
                    pass
            self.publish_status()
            return False
        finally:
            self.operation_lock.release()

    def switch_scenario_callback(self, request, response):
        previous = self.current_scenario_name
        response.success = self.switch(request.scenario_name)
        response.previous_scenario = previous
        response.message = f"Switched to {self.current_scenario_name}" if response.success else self.info
        return response

    def reset_callback(self, request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success, response.message = False, "Scenario operation in progress"
            return response
        try:
            if self.state != "running" or self.ego_spawn is None:
                raise ValueError("load a scenario first")
            self._stop_injection()
            was_paused = self.paused
            old_odom_counter = self.odom_counter
            self.state = "starting"
            self.publish_status()
            self.paused = True
            with self.world_lock:
                ego = self._stop_vehicle()
                if ego is None:
                    raise ValueError("ego vehicle unavailable")
                ego.set_target_velocity(carla.Vector3D())
                ego.set_target_angular_velocity(carla.Vector3D())
                ego.set_transform(self.ego_spawn)
            self.paused = False
            transform = self.ego_spawn
            pose = (transform.location.x, -transform.location.y, -math.radians(transform.rotation.yaw))
            self.wait_localization(pose, old_odom_counter)
            self.state = "running"
            self.paused = was_paused
            self.publish_status()
            response.success, response.message = True, "Ego reset; select a trajectory from the new pose"
        except Exception as error:
            response.success, response.message = False, str(error)
            self.state, self.paused, self.info = "error", True, str(error)
            self.publish_status()
        finally:
            self.operation_lock.release()
        return response

    def reconcile_world_model(self):
        # A world_model container may be started after the environment is ready.
        if self.state != "running" or self.paused or self.bundle is None:
            return
        if not self.operation_lock.acquire(blocking=False):
            return
        try:
            if not self.world_model.needs_reload(self.bundle, self.generation):
                return
            self.world_model.prepare()
            if self.world_model.present:
                from carla_common import find_ego_vehicle
                ego = find_ego_vehicle(self.carla_world, "ego_vehicle")
                transform = ego.get_transform()
                pose = (transform.location.x, -transform.location.y, -math.radians(transform.rotation.yaw))
                self.world_model.apply(self.bundle, pose, self.generation)
        except Exception as error:
            self.info = f"world_model reload failed: {error}"
            self.state, self.paused = "error", True
            with self.world_lock:
                self._stop_vehicle()
            self.publish_status()
            try:
                self._stop_injection()
            except Exception as stop_error:
                self.get_logger().error(f"Could not clear controller input: {stop_error}")
        finally:
            self.operation_lock.release()

    def get_scenarios_callback(self, request, response):
        """Handle get available scenarios service request."""
        response.scenario_names = list(self.available_scenarios.keys())
        response.descriptions = list(self.available_scenarios.values())
        return response

    def _discover_scenarios(self):
        """Discover available scenarios."""
        self.available_scenarios = {key: value.get("description", key)
                                    for key, value in self.registry.scenarios.items()}

    def _unload_scenario(self):
        """Unload current scenario and clean up CARLA world."""
        if self.current_scenario:
            self.current_scenario.cleanup()

        # Clear scenario reference (but keep name until new scenario is set)
        self.current_scenario = None

        # Clean up all spawned actors in CARLA world
        if not self.carla_client:
            return

        try:
            world = self.carla_client.get_world()
            actors = world.get_actors()

            # Stop all controllers first (they need to be stopped before destruction)
            for controller in actors.filter("controller.*"):
                controller.stop()

            # Destroy all spawned actors (excludes static world elements)
            count = 0
            for actor in actors:
                if actor.type_id.startswith(("traffic.", "static.")):
                    continue
                try:
                    actor.destroy()
                    count += 1
                except Exception:
                    pass

            if count > 0:
                self.get_logger().info(f"Cleaned up {count} actors from world")
                if self.get_parameter("synchronous_mode").value:
                    world.tick()
        except Exception as e:
            self.get_logger().warn(f"Error cleaning up world: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioServerNode()

    # Use MultiThreadedExecutor to allow service calls from within callbacks
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=8)
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
