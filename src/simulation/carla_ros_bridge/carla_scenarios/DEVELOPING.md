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

1. Create a new module in `carla_scenarios/scenarios/` (e.g., `my_scenario.py`)
2. Define a class matching the module name in PascalCase (e.g., `MyScenario`)
3. Extend `ScenarioBase`, implement its abstract methods, and declare exactly one map source: `MAP_NAME = "Town10HD_Opt"` for a packaged map or `XODR_FILENAME = "oval.xodr"` for OpenDRIVE
4. Reference by module path: `carla_scenarios.scenarios.my_scenario`

For example:

```python
class MyScenario(ScenarioBase):
    MAP_NAME = "Town10HD_Opt"

    def initialize(self, client):
        return self._initialize_carla(client)

    # Implement get_name(), get_description(), and setup() as usual.
```

Use `switch_scenario` to select both scenario and map. 

## Lifecycle Coordination

When switching scenarios, the server calls the lifecycle_manager's `prepare_for_scenario_switch` service before unloading. This ensures sensors are destroyed before their parent vehicle disappears.

The server publishes `scenario_status` which the lifecycle_manager subscribes to. When status changes to "running" with a new scenario name, the manager brings up all managed nodes.

## Clock Publishing

The server publishes `/clock` from CARLA's simulation timestamp. Nodes should use `use_sim_time:=true` to stay synchronized.

## CARLA API Docs

For more configuration details to implement customizable scenarios, refer to [this link](https://carla.readthedocs.io/en/latest/python_api/#).
