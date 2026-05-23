# Developing dummy_ackermann

## Topics

Both nodes publish to the same topic type with no subscribers.

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `ackermann` | `ackermann_msgs/AckermannDriveStamped` | Generated command |

## Parameters

### ackermann_square_wave_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `period` | double | `10.0` | Full oscillation period (seconds) |
| `amplitude` | double | `0.1` | Steering angle amplitude (radians) |
| `publish_rate` | double | `50.0` | Publish rate (Hz) |

### ackermann_velocity_trapezoid_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `target_velocity` | double | `1.0` | Peak velocity (m/s) |
| `rise_time` | double | `2.0` | Ramp-up duration (seconds) |
| `hold_time` | double | `3.0` | Hold at peak duration (seconds) |
| `ramp_down_time` | double | `2.0` | Ramp-down duration (seconds) |
| `down_hold_time` | double | `1.0` | Hold at zero duration (seconds) |
| `publish_rate` | double | `50.0` | Publish rate (Hz) |

## Build & Launch

```bash
colcon build --packages-select dummy_ackermann
```

Both nodes are launched via `interfacing_can.launch.yaml` with optional arguments:

```bash
# Enable velocity trapezoid test node
ros2 launch interfacing_bringup interfacing_can.launch.yaml test_velocity:=true

# Enable square wave steering test node
ros2 launch interfacing_bringup interfacing_can.launch.yaml test_steering:=true
```
