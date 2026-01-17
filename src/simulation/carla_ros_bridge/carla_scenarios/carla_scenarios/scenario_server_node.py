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
"""Scenario server lifecycle node."""

import importlib
from typing import Optional, Dict
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from carla_msgs.srv import SwitchScenario, GetAvailableScenarios
from carla_msgs.msg import ScenarioStatus
from std_msgs.msg import Header
from std_srvs.srv import Trigger
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time

try:
    import carla
except ImportError:
    carla = None

from carla_scenarios.scenario_base import ScenarioBase


class ScenarioServerNode(LifecycleNode):
    """Lifecycle node for managing CARLA scenarios."""

    def __init__(self, node_name="scenario_server"):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter("carla_host", "localhost")
        self.declare_parameter("carla_port", 2000)
        self.declare_parameter("carla_timeout", 10.0)
        self.declare_parameter(
            "initial_scenario", "carla_scenarios.scenarios.default_scenario"
        )
        # Simulation timing
        self.declare_parameter("carla_fps", 60.0)  # Target simulation FPS
        # Lifecycle manager coordination
        self.declare_parameter("lifecycle_manager_name", "carla_lifecycle_manager")

        # State
        self.carla_client: Optional["carla.Client"] = None
        self.carla_world: Optional["carla.World"] = None
        self.synchronous_mode: bool = False
        self.current_scenario: Optional[ScenarioBase] = None
        self.current_scenario_name: str = ""
        self.available_scenarios: Dict[str, str] = {}

        # ROS interfaces (created in on_configure)
        self.status_publisher = None
        self.clock_publisher = None
        self.switch_scenario_service = None
        self.get_scenarios_service = None
        self.tick_timer = None
        self.prepare_switch_client = None
        self.client_cb_group = None
        self.service_cb_group = None

        self.get_logger().info(f"{node_name} initialized")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info("Configuring...")

        # Get parameters
        host = self.get_parameter("carla_host").value
        port = self.get_parameter("carla_port").value
        timeout = self.get_parameter("carla_timeout").value

        # Connect to CARLA
        if carla is None:
            self.get_logger().error("CARLA Python API not available")
            return TransitionCallbackReturn.FAILURE

        try:
            self.carla_client = carla.Client(host, port)
            self.carla_client.set_timeout(timeout)
            version = self.carla_client.get_server_version()
            self.get_logger().info(f"Connected to CARLA {version} at {host}:{port}")

            # Get world and configure synchronous mode with fixed timestep
            self.carla_world = self.carla_client.get_world()
            carla_fps = self.get_parameter("carla_fps").value

            settings = self.carla_world.get_settings()
            settings.synchronous_mode = True
            settings.fixed_delta_seconds = 1.0 / carla_fps
            self.carla_world.apply_settings(settings)

            self.synchronous_mode = True
            self.get_logger().info(
                f"Configured CARLA: synchronous_mode=True, "
                f"fixed_delta_seconds={settings.fixed_delta_seconds:.4f} ({carla_fps} FPS)"
            )
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
            SwitchScenario, "~/switch_scenario", self.switch_scenario_callback,
            callback_group=self.service_cb_group
        )

        self.get_scenarios_service = self.create_service(
            GetAvailableScenarios,
            "~/get_available_scenarios",
            self.get_scenarios_callback,
            callback_group=self.service_cb_group,
        )

        # Create client for lifecycle manager's prepare_for_scenario_switch service
        lifecycle_manager_name = self.get_parameter("lifecycle_manager_name").value
        self.client_cb_group = rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        self.prepare_switch_client = self.create_client(
            Trigger,
            f"/{lifecycle_manager_name}/prepare_for_scenario_switch",
            callback_group=self.client_cb_group,
        )

        # Discover available scenarios
        self._discover_scenarios()

        self.get_logger().info("Configuration complete")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info("Activating...")

        # Load initial scenario
        initial_scenario = self.get_parameter("initial_scenario").value
        if initial_scenario:
            success = self._load_scenario(initial_scenario)
            if not success:
                self.get_logger().error(
                    f"Failed to load initial scenario: {initial_scenario}"
                )
                return TransitionCallbackReturn.FAILURE

        # Create fast timer for world tick synchronization (1ms)
        # Pedestrian AI needs continuous wait_for_tick() calls to work properly
        # The callback blocks on wait_for_tick(), so this effectively runs every CARLA tick
        self.tick_timer = self.create_timer(0.001, self._tick_callback)

        self.get_logger().info("Activation complete")
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info("Deactivating...")

        # Stop tick timer
        if self.tick_timer:
            self.tick_timer.cancel()
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

        # Disconnect from CARLA
        self.carla_client = None

        self.get_logger().info("Cleanup complete")
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info("Shutting down...")
        return TransitionCallbackReturn.SUCCESS

    def _tick_callback(self):
        """Timer callback for world tick synchronization and status publishing."""
        # Synchronize with world tick
        if self.carla_world:
            try:
                if self.synchronous_mode:
                    self.carla_world.tick()
                else:
                    self.carla_world.wait_for_tick()

                # Publish simulation clock from CARLA timestamp
                if self.clock_publisher:
                    snapshot = self.carla_world.get_snapshot()
                    sim_time = snapshot.timestamp.elapsed_seconds
                    clock_msg = Clock()
                    clock_msg.clock.sec = int(sim_time)
                    clock_msg.clock.nanosec = int((sim_time % 1.0) * 1e9)
                    self.clock_publisher.publish(clock_msg)
            except Exception as e:
                self.get_logger().warn(f"Error ticking world: {e}")

        # Execute scenario logic
        if self.current_scenario:
            try:
                self.current_scenario.execute()
            except Exception as e:
                self.get_logger().error(f"Error executing scenario: {e}")

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

    def switch_scenario_callback(self, request, response):
        """Handle switch scenario service request."""
        self.get_logger().info(f"Received switch_scenario request: {request.scenario_name}")

        previous = self.current_scenario_name

        # Request lifecycle manager to cleanup managed nodes first
        if self.prepare_switch_client and self.current_scenario:
            if self.prepare_switch_client.service_is_ready():
                self.get_logger().info("Requesting lifecycle manager to cleanup nodes...")
                try:
                    result = self.prepare_switch_client.call(Trigger.Request())
                    if result.success:
                        self.get_logger().info("Lifecycle manager cleanup complete")
                    else:
                        self.get_logger().warn(f"Lifecycle manager cleanup failed: {result.message}")
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
        # Built-in scenarios
        builtin_scenarios = {
            "carla_scenarios.scenarios.default_scenario": "Default Ego Spawn",
            "carla_scenarios.scenarios.empty_scenario": "Empty World (no NPCs)",
            "carla_scenarios.scenarios.light_traffic_scenario": "Light Traffic",
            "carla_scenarios.scenarios.heavy_traffic_scenario": "Heavy Traffic",
        }
        self.available_scenarios.update(builtin_scenarios)

        # TODO: Discover external scenarios via entry points
        # from importlib.metadata import entry_points
        # discovered = entry_points(group='carla_scenarios.plugins')
        # for ep in discovered:
        #     self.available_scenarios[ep.name] = ep.value

        self.get_logger().info(f"Discovered {len(self.available_scenarios)} scenarios")

    def _unload_scenario(self):
        """Unload current scenario and clean up CARLA world."""
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
                world.tick()
        except Exception as e:
            self.get_logger().warn(f"Error cleaning up world: {e}")

    def _load_scenario(self, scenario_module_path: str) -> bool:
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
            scenario.logger = self.get_logger()
            if not scenario.initialize(self.carla_client):
                self.get_logger().error(
                    f"Failed to initialize scenario: {scenario_module_path}"
                )
                return False

            if not scenario.setup():
                self.get_logger().error(
                    f"Failed to setup scenario: {scenario_module_path}"
                )
                return False

            # Refresh world reference (scenario may have changed the map)
            self.carla_world = self.carla_client.get_world()
            self.synchronous_mode = self.carla_world.get_settings().synchronous_mode

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
