"""Scenario server lifecycle node."""
import importlib
from typing import Optional, Dict
import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from carla_msgs.srv import SwitchScenario, GetAvailableScenarios
from carla_msgs.msg import ScenarioStatus
from std_msgs.msg import Header

try:
    import carla
except ImportError:
    carla = None

from carla_scenarios.scenario_base import ScenarioBase


class ScenarioServerNode(LifecycleNode):
    """Lifecycle node for managing CARLA scenarios."""

    def __init__(self, node_name='scenario_server'):
        super().__init__(node_name)

        # CARLA connection parameters
        self.declare_parameter('carla_host', 'localhost')
        self.declare_parameter('carla_port', 2000)
        self.declare_parameter('carla_timeout', 10.0)
        self.declare_parameter('initial_scenario', 'carla_scenarios.scenarios.default_scenario')

        # State
        self.carla_client: Optional['carla.Client'] = None
        self.current_scenario: Optional[ScenarioBase] = None
        self.current_scenario_name: str = ""
        self.available_scenarios: Dict[str, str] = {}

        # ROS interfaces (created in on_configure)
        self.status_publisher = None
        self.switch_scenario_service = None
        self.get_scenarios_service = None
        self.scenario_timer = None

        self.get_logger().info(f'{node_name} initialized')

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure lifecycle callback."""
        self.get_logger().info('Configuring...')

        # Get parameters
        host = self.get_parameter('carla_host').value
        port = self.get_parameter('carla_port').value
        timeout = self.get_parameter('carla_timeout').value

        # Connect to CARLA
        if carla is None:
            self.get_logger().error('CARLA Python API not available')
            return TransitionCallbackReturn.FAILURE

        try:
            self.carla_client = carla.Client(host, port)
            self.carla_client.set_timeout(timeout)
            version = self.carla_client.get_server_version()
            self.get_logger().info(f'Connected to CARLA {version} at {host}:{port}')
        except Exception as e:
            self.get_logger().error(f'Failed to connect to CARLA: {e}')
            return TransitionCallbackReturn.FAILURE

        # Create ROS interfaces
        self.status_publisher = self.create_lifecycle_publisher(
            ScenarioStatus,
            '~/scenario_status',
            10
        )

        self.switch_scenario_service = self.create_service(
            SwitchScenario,
            '~/switch_scenario',
            self.switch_scenario_callback
        )

        self.get_scenarios_service = self.create_service(
            GetAvailableScenarios,
            '~/get_available_scenarios',
            self.get_scenarios_callback
        )

        # Discover available scenarios
        self._discover_scenarios()

        self.get_logger().info('Configuration complete')
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activate lifecycle callback."""
        self.get_logger().info('Activating...')

        # Load initial scenario
        initial_scenario = self.get_parameter('initial_scenario').value
        if initial_scenario:
            success = self._load_scenario(initial_scenario)
            if not success:
                self.get_logger().error(f'Failed to load initial scenario: {initial_scenario}')
                return TransitionCallbackReturn.FAILURE

        # Create timer for scenario execution
        self.scenario_timer = self.create_timer(0.1, self.scenario_timer_callback)

        self.get_logger().info('Activation complete')
        return super().on_activate(state)

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Deactivate lifecycle callback."""
        self.get_logger().info('Deactivating...')

        # Stop scenario execution
        if self.scenario_timer:
            self.scenario_timer.cancel()
            self.scenario_timer = None

        # Cleanup current scenario
        if self.current_scenario:
            self.current_scenario.cleanup()
            self.current_scenario = None
            self.current_scenario_name = ""

        self.get_logger().info('Deactivation complete')
        return super().on_deactivate(state)

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Cleanup lifecycle callback."""
        self.get_logger().info('Cleaning up...')

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

        # Disconnect from CARLA
        self.carla_client = None

        self.get_logger().info('Cleanup complete')
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Shutdown lifecycle callback."""
        self.get_logger().info('Shutting down...')
        return TransitionCallbackReturn.SUCCESS

    def scenario_timer_callback(self):
        """Periodically execute scenario and publish status."""
        if self.current_scenario:
            try:
                self.current_scenario.execute()
            except Exception as e:
                self.get_logger().error(f'Error executing scenario: {e}')

        # Publish status
        if self.status_publisher and self.status_publisher.is_activated:
            status_msg = ScenarioStatus()
            status_msg.header = Header()
            status_msg.header.stamp = self.get_clock().now().to_msg()
            status_msg.scenario_name = self.current_scenario_name
            status_msg.description = self.current_scenario.get_description() if self.current_scenario else ""
            status_msg.state = "running" if self.current_scenario else "idle"
            status_msg.info = ""
            self.status_publisher.publish(status_msg)

    def switch_scenario_callback(self, request, response):
        """Handle switch scenario service request."""
        self.get_logger().info(f'Switching to scenario: {request.scenario_name}')

        previous = self.current_scenario_name
        success = self._load_scenario(request.scenario_name)

        response.success = success
        response.previous_scenario = previous
        response.message = f'Switched to {request.scenario_name}' if success else 'Failed to switch scenario'

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
            'carla_scenarios.scenarios.default_scenario': 'Default Ego Spawn',
        }
        self.available_scenarios.update(builtin_scenarios)

        # TODO: Discover external scenarios via entry points
        # from importlib.metadata import entry_points
        # discovered = entry_points(group='carla_scenarios.plugins')
        # for ep in discovered:
        #     self.available_scenarios[ep.name] = ep.value

        self.get_logger().info(f'Discovered {len(self.available_scenarios)} scenarios')

    def _cleanup_world(self):
        """Clean up all spawned actors in the CARLA world."""
        try:
            world = self.carla_client.get_world()
            actors = world.get_actors()
            vehicles = actors.filter('vehicle.*')
            sensors = actors.filter('sensor.*')

            # Destroy all vehicles and sensors
            for actor in list(vehicles) + list(sensors):
                actor.destroy()

            count = len(vehicles) + len(sensors)
            if count > 0:
                self.get_logger().info(f'Cleaned up {count} actors from world')
        except Exception as e:
            self.get_logger().warn(f'Error cleaning up world: {e}')

    def _load_scenario(self, scenario_module_path: str) -> bool:
        """Load and initialize a scenario."""
        # Cleanup previous scenario
        if self.current_scenario:
            self.current_scenario.cleanup()
            self.current_scenario = None

        # Clean up any remaining actors in the world
        self._cleanup_world()

        try:
            # Import scenario module
            # scenario_module_path: e.g. 'carla_scenarios.scenarios.default_scenario'
            # class_name derived from last part: 'default_scenario' -> 'DefaultScenario'
            parts = scenario_module_path.rsplit('.', 1)
            if len(parts) != 2:
                self.get_logger().error(f'Invalid scenario path: {scenario_module_path}')
                return False

            class_name = parts[1]
            # Convert module name to class name (e.g., default_scenario -> DefaultScenario)
            if not class_name[0].isupper():
                class_name = ''.join(word.capitalize() for word in class_name.split('_'))

            module = importlib.import_module(scenario_module_path)
            scenario_class = getattr(module, class_name)

            # Instantiate and initialize scenario
            scenario = scenario_class()
            if not scenario.initialize(self.carla_client):
                self.get_logger().error(f'Failed to initialize scenario: {scenario_module_path}')
                return False

            if not scenario.setup():
                self.get_logger().error(f'Failed to setup scenario: {scenario_module_path}')
                scenario.cleanup()
                return False

            self.current_scenario = scenario
            self.current_scenario_name = scenario_module_path
            self.get_logger().info(f'Loaded scenario: {scenario.get_name()}')
            return True

        except Exception as e:
            self.get_logger().error(f'Error loading scenario {scenario_module_path}: {e}')
            return False


def main(args=None):
    rclpy.init(args=args)
    node = ScenarioServerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
