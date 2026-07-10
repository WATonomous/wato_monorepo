# Developing carla_scenarios

## Design Rationale

The scenario system separates "what world state to create" (scenarios) from "how to manage that state" (scenario_server). This allows adding new scenarios without modifying the server.

Scenarios are loaded dynamically by module path so users can define custom scenarios in their own packages without forking this one.

## Architecture

- **scenario_server** - Lifecycle node that owns the CARLA connection, ticks the world, and coordinates with the lifecycle_manager
- **scenario_base** - Abstract base class defining the scenario interface
- **scenarios/** - Concrete scenario implementations

The server is the only node that calls `world.tick()` in synchronous mode. All other nodes just read from CARLA on their own timers.

## Adding a New Scenario

### In this package

1. Create a new module in `carla_scenarios/scenarios/` (e.g., `my_scenario.py`)
2. Define a class matching the module name in PascalCase (e.g., `MyScenario`)
3. Extend `ScenarioBase` and implement `setup()` at minimum
4. Register it in `setup.py` under the `carla_scenarios.plugins` entry point group:
   ```python
   "carla_scenarios.plugins": [
       "my_scenario = carla_scenarios.scenarios.my_scenario:MyScenario",
   ],
   ```

### In an external package

Any ROS 2 Python package can contribute scenarios without modifying this package.

1. Create your scenario class extending `ScenarioBase`:
   ```python
   # my_pkg/my_pkg/scenarios/my_scenario.py
   from carla_scenarios.scenario_base import ScenarioBase

   class MyScenario(ScenarioBase):
       def get_name(self): return "My Scenario"
       def get_description(self): return "Does something interesting"
       def setup(self): ...
   ```

2. Register it in your package's `setup.py`:
   ```python
   entry_points={
       "carla_scenarios.plugins": [
           "my_scenario = my_pkg.scenarios.my_scenario:MyScenario",
       ],
   },
   ```

3. After installing your package (`colcon build`), the scenario server will automatically discover it on startup and include it in `get_available_scenarios` responses.

4. Load it via the `switch_scenario` service using its full module path:
   ```
   my_pkg.scenarios.my_scenario
   ```

The base class provides `_initialize_carla()` and `_load_map()` helpers to reduce boilerplate.

## Lifecycle Coordination

When switching scenarios, the server calls the lifecycle_manager's `prepare_for_scenario_switch` service before unloading. This ensures sensors are destroyed before their parent vehicle disappears.

The server publishes `scenario_status` which the lifecycle_manager subscribes to. When status changes to "running" with a new scenario name, the manager brings up all managed nodes.

## Clock Publishing

The server publishes `/clock` from CARLA's simulation timestamp. Nodes should use `use_sim_time:=true` to stay synchronized.
