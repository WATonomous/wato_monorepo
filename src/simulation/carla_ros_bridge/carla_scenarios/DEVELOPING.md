# Developing carla_scenarios

## Design Rationale

The scenario system separates "what world state to create" (scenarios) from "how to manage that state" (scenario_server). This allows adding new scenarios without modifying the server.

Scenarios are loaded dynamically by module path so users can define custom scenarios in their own packages without forking this one.

## Architecture

- **scenario_server** - Lifecycle node that owns the CARLA connection, ticks the world, and coordinates with the lifecycle_manager
- **scenario_base** - Abstract base class defining the scenario interface
- **scenarios/** - Concrete scenario implementations

The server is the only node that calls `world.tick()` in synchronous mode. All other nodes just read from CARLA on their own timers.

## Adding a Built-in Scenario

1. Create a new module in `carla_scenarios/scenarios/` (e.g., `my_scenario.py`)
2. Define a class matching the module name in PascalCase (e.g., `MyScenario`)
3. Extend `ScenarioBase` and implement `setup()` at minimum
4. Register it in `setup.py` under the `carla_scenarios.scenarios` entry point group
5. Add it to the `builtin_scenarios` dict in `_discover_scenarios()` in `scenario_server_node.py`
6. Reference by module path: `carla_scenarios.scenarios.my_scenario`

The base class provides `_initialize_carla()` and `_load_map()` helpers to reduce boilerplate.

## Creating Custom Scenarios in an External Package

You can define scenarios in your own ROS package without forking `carla_scenarios`. The scenario server discovers external scenarios at startup via Python entry points.

### 1. Create your ROS package

```bash
ros2 pkg create my_custom_scenarios --build-type ament_python --dependencies carla_scenarios
```

### 2. Write your scenario

Create a module (e.g., `my_custom_scenarios/scenarios/highway_scenario.py`):

```python
from carla_scenarios.scenario_base import ScenarioBase
import carla

class HighwayScenario(ScenarioBase):
    def get_name(self) -> str:
        return "Highway Scenario"

    def get_description(self) -> str:
        return "Highway driving with moderate traffic"

    def initialize(self, client: carla.Client) -> bool:
        return self._initialize_carla(client)

    def setup(self) -> bool:
        if self.world is None:
            return False
        self._load_map("Town04")
        spawn_points = self.world.get_map().get_spawn_points()
        if not spawn_points:
            return False
        self.spawn_ego_vehicle(spawn_points[0])
        self.spawn_vehicles(15, autopilot=True)
        return True
```

**Naming convention:** The class name must be the PascalCase version of the module name. `highway_scenario.py` must contain `HighwayScenario`.

### 3. Register the entry point

In your package's `setup.py`, add a `carla_scenarios.scenarios` entry point:

```python
setup(
    name="my_custom_scenarios",
    # ...
    install_requires=["setuptools"],
    entry_points={
        "carla_scenarios.scenarios": [
            "highway = my_custom_scenarios.scenarios.highway_scenario",
        ],
    },
)
```

The entry point name (e.g., `highway`) is a human-readable key. The value is the full dotted module path.

### 4. Build and run

```bash
colcon build --packages-select my_custom_scenarios
source install/setup.bash
```

The scenario server will discover your scenario on startup. You can verify with:

```bash
ros2 service call /scenario_server/get_available_scenarios carla_msgs/srv/GetAvailableScenarios
```

To load it:

```bash
ros2 service call /scenario_server/switch_scenario carla_msgs/srv/SwitchScenario \
  "{scenario_name: 'my_custom_scenarios.scenarios.highway_scenario'}"
```

Or set it as the initial scenario via parameter:

```bash
ros2 run carla_scenarios scenario_server --ros-args \
  -p initial_scenario:=my_custom_scenarios.scenarios.highway_scenario
```

## Lifecycle Coordination

When switching scenarios, the server calls the lifecycle_manager's `prepare_for_scenario_switch` service before unloading. This ensures sensors are destroyed before their parent vehicle disappears.

The server publishes `scenario_status` which the lifecycle_manager subscribes to. When status changes to "running" with a new scenario name, the manager brings up all managed nodes.

## Clock Publishing

The server publishes `/clock` from CARLA's simulation timestamp. Nodes should use `use_sim_time:=true` to stay synchronized.
