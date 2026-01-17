# Developing carla_scenarios

## Architecture

Scenarios are Python classes that extend `ScenarioBase`. The scenario server dynamically loads scenarios by module path.

## ScenarioBase Interface

```python
class ScenarioBase:
    def initialize(self, client: carla.Client) -> bool:
        """Called once with CARLA client."""

    def setup(self) -> bool:
        """Set up the scenario (spawn actors, set weather, etc.)."""

    def execute(self) -> None:
        """Called every tick while scenario is running."""

    def cleanup(self) -> None:
        """Clean up spawned actors."""

    def get_name(self) -> str:
        """Return scenario display name."""

    def get_description(self) -> str:
        """Return scenario description."""
```

## Creating a New Scenario

1. Create module in `carla_scenarios/scenarios/`:

   ```python
   from carla_scenarios.scenario_base import ScenarioBase

   class MyScenario(ScenarioBase):
       def setup(self):
           # Spawn actors, configure world
           return True
   ```

2. Class name must match module name in PascalCase:
   - `my_scenario.py` → `MyScenario`

3. Reference by module path:
   - `carla_scenarios.scenarios.my_scenario`

## Scenario Loading

The server converts module path to class:

```python
module = importlib.import_module(scenario_module_path)
class_name = "".join(word.capitalize() for word in module_name.split("_"))
scenario_class = getattr(module, class_name)
```

## World Cleanup

Before loading a new scenario, the server:
1. Calls `cleanup()` on current scenario
2. Destroys all spawned actors (except static world elements)
3. Ticks the world to process destruction

## Tick Synchronization

The server calls `world.wait_for_tick()` continuously. This is required for pedestrian AI to function properly.

## NPC Traffic

For autopilot vehicles, use CARLA's Traffic Manager:

```python
vehicle.set_autopilot(True)
```

For pedestrians, spawn `controller.ai.walker`:

```python
controller = world.spawn_actor(controller_bp, transform, attach_to=walker)
controller.start()
controller.go_to_location(destination)
```
