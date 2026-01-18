# Developing carla_control

## Design Rationale

Control nodes run a fast timer loop (50 Hz default) rather than applying commands directly in the subscription callback. This:
1. Ensures consistent control rate regardless of command publish rate
2. Enables command timeout detection (stop vehicle if commands stop arriving)
3. Decouples ROS timing from CARLA timing

## Architecture

1. Subscription callback stores latest command with timestamp
2. Control timer runs at `control_rate` Hz
3. Timer checks if command is stale (older than `command_timeout`)
4. If stale: brake to stop. If fresh: apply command to vehicle

## Ego Vehicle Discovery

Vehicles are found by the `role_name` attribute set when spawning in CARLA. The scenario is responsible for spawning the ego vehicle with the correct role_name.

If no matching vehicle is found, the node fails to activate rather than silently using a wrong vehicle.

## Command Timeout Safety

The timeout prevents runaway vehicles when:
- Teleop source disconnects
- Planning node crashes
- Network issues interrupt command flow

Timeout is checked every control loop iteration. When triggered, the vehicle brakes to a full stop.

## Adding a New Control Node

1. Extend LifecycleNode
2. On configure: connect to CARLA, find ego vehicle by role_name
3. On activate: create control timer
4. On deactivate: brake vehicle, destroy timer
5. On cleanup: release CARLA resources
6. Implement command timeout in timer callback
