# Developing carla_teleop

## Design Rationale

Teleop uses `geometry_msgs/Twist` rather than a CARLA-specific message for compatibility with:
- Foxglove teleop panel
- teleop_twist_keyboard
- Joystick drivers (joy_teleop)
- Any standard ROS teleop source

This lets users control the CARLA vehicle with their existing teleop tools.

## Twist to VehicleControl Mapping

- `linear.x` is normalized by `max_speed` to get throttle (0-1)
- Negative `linear.x` engages reverse
- `angular.z` is normalized by `max_steering` to get steer (-1 to 1)
- Steering is inverted: positive angular.z (left turn) becomes negative CARLA steer

Scale factors (`throttle_scale`, `steering_scale`) allow tuning response.

## Autonomy Mode

The `set_autonomy` service toggles CARLA's built-in autopilot. When enabled:
1. `vehicle.set_autopilot(True)` activates Traffic Manager control
2. Teleop commands are ignored
3. Vehicle follows traffic rules and avoids collisions

When disabled:
1. Autopilot is deactivated
2. Immediate brake is applied (Traffic Manager may have been accelerating)
3. Teleop commands resume control

## Command Timeout

Same pattern as ackermann_control - if no Twist is received within `command_timeout`, brake to stop.
