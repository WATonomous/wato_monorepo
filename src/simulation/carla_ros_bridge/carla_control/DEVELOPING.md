# Developing carla_control

## Architecture

All control nodes follow the lifecycle node pattern for coordinated startup with the CARLA simulation.

## Lifecycle Node Pattern

```python
class ControlNode(LifecycleNode):
    def on_configure(self, state):
        # Connect to CARLA
        # Find ego vehicle by role_name
        # Create subscriptions

    def on_activate(self, state):
        # Create control timer

    def on_deactivate(self, state):
        # Stop vehicle (brake)
        # Destroy timer

    def on_cleanup(self, state):
        # Release CARLA resources
```

## Control Flow

1. **Command Callback** - Stores latest command and timestamp
2. **Control Timer** (50 Hz) - Applies command or stops vehicle if timed out

## Command Timeout

Nodes implement a safety timeout. If no command is received within `command_timeout` seconds, the vehicle brakes to a stop.

## Ego Vehicle Discovery

Vehicles are found by `role_name` attribute:
```python
ego_vehicles = [v for v in vehicles
                if v.attributes.get("role_name") == role_name]
```

If no matching vehicle is found, falls back to the first available vehicle.

## Adding a New Control Node

1. Extend `LifecycleNode`
2. Implement all lifecycle callbacks
3. Use the control timer pattern for applying commands
4. Implement command timeout safety
5. Add entry point in `setup.py`
