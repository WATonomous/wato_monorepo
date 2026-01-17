# Developing carla_teleop

## Architecture

The teleop node converts `geometry_msgs/Twist` to CARLA `VehicleControl`, providing compatibility with standard ROS teleop interfaces.

## Twist to VehicleControl Mapping

```python
# linear.x → throttle/brake
normalized_speed = twist.linear.x / max_speed
if normalized_speed >= 0:
    control.throttle = normalized_speed * throttle_scale
    control.reverse = False
else:
    control.throttle = abs(normalized_speed) * throttle_scale
    control.reverse = True

# angular.z → steering (inverted: positive twist = left = negative CARLA steer)
control.steer = -twist.angular.z / max_steering * steering_scale
```

## Autonomy Mode

When autonomy is enabled:
1. CARLA autopilot is activated via `vehicle.set_autopilot(True)`
2. Teleop commands are ignored
3. Vehicle follows Traffic Manager behavior

When autonomy is disabled:
1. Autopilot is deactivated
2. Immediate brake is applied to take control from Traffic Manager
3. Teleop commands resume

## Command Timeout

If no Twist message is received within `command_timeout` seconds, the vehicle brakes to a stop. This prevents runaway vehicles if the teleop source disconnects.

## Control Loop

The control timer runs at 50 Hz:
1. Check if autonomy is enabled (skip if yes)
2. Check command timeout
3. Convert latest Twist to VehicleControl
4. Apply to vehicle

## Foxglove Integration

The node is designed for Foxglove's teleop panel which publishes `geometry_msgs/Twist`:
- Joystick forward/back → `linear.x`
- Joystick left/right → `angular.z`
