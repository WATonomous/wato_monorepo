# carla_control

Vehicle control nodes for CARLA simulation.

## Nodes

### ackermann_control

Receives `AckermannDriveStamped` commands and applies them to the ego vehicle in CARLA using CARLA's native Ackermann control API. The node finds the ego vehicle by its `role_name` attribute and applies speed, acceleration, jerk, and steering commands directly.

If no command is received within the `command_timeout`, the vehicle automatically brakes to a stop. This prevents runaway vehicles when the control source disconnects.

```bash
ros2 run carla_control ackermann_control
```

**Subscriptions:** `~/command` (`ackermann_msgs/AckermannDriveStamped`)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `carla_host` | string | `localhost` | CARLA server hostname |
| `carla_port` | int | `2000` | CARLA server port |
| `carla_timeout` | double | `10.0` | Connection timeout in seconds |
| `role_name` | string | `ego_vehicle` | Role name of the ego vehicle to control |
| `command_timeout` | double | `0.5` | Stop vehicle if no command received within this time (seconds) |
| `control_rate` | double | `50.0` | Control loop frequency in Hz |