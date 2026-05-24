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

## After Launching

Confirm the node is publishing at the expected rate:

```bash
ros2 topic hz /ackermann          # expect ~50 Hz
ros2 topic echo /ackermann        # watch values change over time
```

## Definition of Good Result

**ackermann_velocity_trapezoid_node** — with defaults (`target_velocity=1.0`, `rise_time=2.0`, `hold_time=3.0`, `ramp_down_time=2.0`, `down_hold_time=1.0`), the `speed` field should follow this profile over each 8-second cycle:

| Time | Expected speed |
|------|---------------|
| 0–2 s | Ramps 0.0 → 1.0 m/s linearly |
| 2–5 s | Holds at 1.0 m/s |
| 5–7 s | Ramps 1.0 → 0.0 m/s linearly |
| 7–8 s | Holds at 0.0 m/s |

**ackermann_square_wave_node** — with defaults (`period=10.0`, `amplitude=0.1`), `steering_angle` should alternate between `+0.1` and `−0.1` rad every 5 seconds, with `speed = 0.0` throughout.
