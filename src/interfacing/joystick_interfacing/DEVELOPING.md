# Developing joystick_interfacing

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `joy` | `sensor_msgs/Joy` | Raw joystick axes and buttons from `joy_node` |
| `oscc_interfacing/is_armed` | `std_msgs/Bool` | Current vehicle arming state |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `joystick/ackermann` | `ackermann_msgs/AckermannDriveStamped` | Ackermann commands (active in ACKERMANN mode) |
| `roscco` | `roscco_msg/Roscco` | ROSCCO commands (active in ROSCCO mode) |
| `joystick/is_idle` | `std_msgs/Bool` | `true` when enable axis is not pressed |
| `joystick/state` | `std_msgs/Int8` | Current mode: 0 = NULL, 1 = ACKERMANN, 2 = ROSCCO |
| `joy/set_feedback` | `sensor_msgs/JoyFeedback` | Haptic rumble commands |

### Service Clients

| Service | Type | Description |
|---------|------|-------------|
| `/oscc_interfacing/arm` | `std_srvs/SetBool` | Arm (`true`) or disarm (`false`) the vehicle |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_axis` | int | `2` | Axis index for enable trigger (must be ≤ −0.9 to enable) |
| `toggle_button` | int | `0` | Button index to toggle ACKERMANN ↔ ROSCCO mode |
| `arming_button` | int | `0` | Button index to arm/disarm the vehicle |
| `steering_axis` | int | — | Axis index for steering (left/right) |
| `throttle_axis` | int | — | Axis index for throttle (forward/backward) |
| `ackermann_max_speed` | double | `2.0` | Max speed in ACKERMANN mode (m/s) |
| `ackermann_max_steering_angle` | double | `0.5` | Max steering in ACKERMANN mode (radians) |
| `roscco_max_speed` | double | `1.0` | Max speed in ROSCCO mode (m/s) |
| `roscco_max_steering_angle` | double | `0.3` | Max steering in ROSCCO mode (radians) |
| `invert_steering` | bool | `false` | Invert steering direction |
| `invert_throttle` | bool | `false` | Invert throttle direction |
| `toggle_vibration_intensity` | double | `0.5` | Rumble intensity (0.0–1.0) |
| `toggle_vibration_duration_ms` | int | `100` | Rumble pulse duration (ms) |

## Build & Launch

```bash
colcon build --packages-select joystick_interfacing
ros2 launch joystick_interfacing joystick_interfacing.launch.yaml
```

## Finding Axis and Button Indices

Run `joy_node` and echo the `/joy` topic while pressing buttons and moving axes:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

Triggers are typically reported as axes (ranging −1.0 to 1.0), not buttons. The enable axis threshold of −0.9 is designed for trigger axes that default to +1.0 (unpressed) and go to −1.0 (fully pressed).
