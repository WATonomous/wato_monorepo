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

## Internal Architecture

**Enable axis:** The node checks `joy.axes[enable_axis]` on every joy message. If the value is > −0.9, the operator is considered not engaged and the node outputs zero commands. This threshold is designed for trigger axes that report +1.0 when unpressed and −1.0 when fully pressed.

**Mode state machine:** Three states: `NULL` (no valid mode set), `ACKERMANN` (publishes on `joystick/ackermann`), `ROSCCO` (publishes on `roscco`). Pressing `toggle_button` cycles between ACKERMANN and ROSCCO. The haptic pulse count (1 pulse → ACKERMANN, 2 pulses → ROSCCO) confirms the new state.

**Command scaling:** Stick axes (−1.0 to +1.0) are linearly scaled by `ackermann_max_steering_angle` and `ackermann_max_speed` (or their ROSCCO equivalents). The `invert_steering` / `invert_throttle` flags negate the axis value before scaling.

## After Launching

1. **Verify joystick is detected:**

   ```bash
   ros2 topic hz /joy   # should publish when any axis/button changes
   ```

2. **Hold the enable trigger** (axis index `enable_axis`, default: axis 2) and move the steering/throttle sticks. Verify commands publish:

```bash
   ros2 topic echo /joystick/ackermann
   ros2 topic echo /joystick/is_idle   # should be false while enable held
   ```

1. **Release enable trigger** — `is_idle` should go `true` and commands should go to zero.

2. **Test mode toggle** — press `toggle_button` (default: button 0). The joystick should vibrate (2 pulses → ROSCCO, 1 pulse → ACKERMANN) and `/joystick/state` should change value.

3. **Test arming** — press `arming_button`. The node calls `/oscc_interfacing/arm`. If OSCC is running, expect 2 vibration pulses on success.

## Definition of Good Result

| Check | Expected |
|-------|----------|
| Enable held, stick at max | `steering_angle` = `±ackermann_max_steering_angle` (default ±0.5 rad) |
| Enable held, throttle at max | `speed` = `ackermann_max_speed` (default 2.0 m/s) |
| Enable released | `speed = 0.0`, `steering_angle = 0.0`, `is_idle = true` |
| Mode toggle to ROSCCO | `state = 2`, 2 vibration pulses |
| Mode toggle to ACKERMANN | `state = 1`, 1 vibration pulse |
| Arm success | 2 vibration pulses, `/oscc_interfacing/is_armed = true` |

## Finding Axis and Button Indices

Run `joy_node` and echo the `/joy` topic while pressing buttons and moving axes:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

Triggers are typically reported as axes (ranging −1.0 to 1.0), not buttons. The enable axis threshold of −0.9 is designed for trigger axes that default to +1.0 (unpressed) and go to −1.0 (fully pressed).
