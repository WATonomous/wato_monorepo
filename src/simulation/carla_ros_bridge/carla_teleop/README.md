# carla_teleop

Teleoperation control for CARLA vehicles.

## Nodes

### teleop

Receives `Twist` commands (compatible with Foxglove teleop panel, `teleop_twist_keyboard`, joystick drivers, etc.) and converts them to CARLA vehicle control. `linear.x` maps to throttle/brake and `angular.z` maps to steering.

The node also provides a `set_autonomy` service to toggle CARLA's built-in autopilot. When autonomy is enabled, teleop commands are ignored and the vehicle follows CARLA's Traffic Manager AI.

If no command is received within the `command_timeout`, the vehicle automatically brakes to a stop.

```bash
ros2 run carla_teleop teleop
```

**Subscriptions:** `cmd_vel` (`geometry_msgs/Twist`)

**Services:** `set_autonomy` (`std_srvs/SetBool`) - Enable/disable CARLA autopilot

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `carla_host` | string | `localhost` | CARLA server hostname |
| `carla_port` | int | `2000` | CARLA server port |
| `carla_timeout` | double | `10.0` | Connection timeout in seconds |
| `role_name` | string | `ego_vehicle` | Role name of the ego vehicle to control |
| `max_speed` | double | `10.0` | Max speed for normalizing linear.x input (m/s) |
| `max_steering` | double | `1.0` | Max angular.z value for normalizing steering input |
| `throttle_scale` | double | `1.0` | Scale factor applied to throttle output |
| `steering_scale` | double | `1.0` | Scale factor applied to steering output |
| `command_timeout` | double | `0.5` | Stop vehicle if no command received within this time (seconds) |
| `control_rate` | double | `50.0` | Control loop frequency in Hz |
