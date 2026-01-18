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
3. Extend `ScenarioBase` and implement `setup()` at minimum
4. Reference by module path: `carla_scenarios.scenarios.my_scenario`

The base class provides `_initialize_carla()` and `_load_map()` helpers to reduce boilerplate.

## Lifecycle Coordination

When switching scenarios, the server calls the lifecycle_manager's `prepare_for_scenario_switch` service before unloading. This ensures sensors are destroyed before their parent vehicle disappears.

The server publishes `scenario_status` which the lifecycle_manager subscribes to. When status changes to "running" with a new scenario name, the manager brings up all managed nodes.

## Clock Publishing

The server publishes `/clock` from CARLA's simulation timestamp. Nodes should use `use_sim_time:=true` to stay synchronized.
