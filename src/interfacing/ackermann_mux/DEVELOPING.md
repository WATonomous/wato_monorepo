# Developing ackermann_mux

## Topics

### Subscribed

Input topics are declared dynamically from the `inputs` parameter at configure time.

| Topic | Type | Description |
|-------|------|-------------|
| `inputs[*].topic` | `ackermann_msgs/AckermannDriveStamped` | Command from each configured source |
| `inputs[*].mask_topic` | `std_msgs/Bool` | Per-input mask; `true` = input is idle/disabled |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `ackermann` | `ackermann_msgs/AckermannDriveStamped` | Selected command output |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `safety_threshold` | double | `0.5` | Max seconds since last command before a safety-gated input triggers emergency |
| `publish_rate_hz` | double | `50.0` | Output publish rate (Hz) |
| `emergency.steering_angle` | float | `0.0` | Emergency steering angle (radians) |
| `emergency.speed` | float | `-0.5` | Emergency speed (m/s); negative = brake |
| `inputs.<name>.topic` | string | — | Input topic name |
| `inputs.<name>.priority` | int | — | Priority level; higher number wins |
| `inputs.<name>.has_mask` | bool | `false` | Whether this input has a mask topic |
| `inputs.<name>.mask_topic` | string | `""` | Mask topic name (required if `has_mask: true`) |
| `inputs.<name>.safety_gating` | bool | `false` | Trigger emergency if this input goes stale |

## Example Configuration

```yaml
ackermann_mux:
  ros__parameters:
    safety_threshold: 0.5
    publish_rate_hz: 50.0
    emergency:
      steering_angle: 0.0
      speed: -0.5
    inputs:
      joystick:
        topic: /joystick/ackermann
        priority: 200
        has_mask: true
        mask_topic: /joystick/ackermann_is_idle
        safety_gating: true
      action:
        topic: /action/ackermann
        priority: 100
        has_mask: false
        safety_gating: false
```

## Build & Launch

```bash
colcon build --packages-select ackermann_mux
ros2 launch ackermann_mux ackermann_mux.launch.yaml
```

## Internal Architecture

The node is a lifecycle node managed by `wato_lifecycle_manager`. It runs a single wall timer on the ROS executor — no background threads.

**Arbitration logic (per publish tick):**
1. For each safety-gated input, compare `now - last_received_time` against `safety_threshold`. If any exceeds the threshold, publish emergency and return.
2. Iterate all inputs sorted by descending priority.
3. Skip inputs that have never received a message or whose mask topic last published `true`.
4. Publish the latest command from the first non-skipped input. If none, publish emergency.

**Mask vs safety gating:** Masking is voluntary (the source says "ignore me"). Safety gating is watchdog-style (the mux stops waiting for a source that has gone silent). Use masking for joystick idle; use safety gating for the joystick overall to catch driver disconnects.

**Publish model:** The output timer runs at `publish_rate_hz` (default 50 Hz) regardless of input arrival — the mux always sends the most recently cached command and never blocks waiting for new input. Increase `publish_rate_hz` if downstream nodes need a faster command stream; decrease `safety_threshold` to react faster to stale inputs.

## After Launching

1. **Verify output is publishing:**

   ```bash
   ros2 topic hz /ackermann   # expect 50 Hz
   ```

2. **Test priority arbitration** — publish a command on a low-priority input and confirm it appears on `/ackermann`:

```bash
   ros2 topic pub /action/ackermann ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 1.0, steering_angle: 0.1}}" --rate 10
   ros2 topic echo /ackermann --once
   ```

1. **Test override** — while the low-priority input is publishing, publish on the joystick input and confirm it takes over immediately:

   ```bash
ros2 topic pub /joystick/ackermann ackermann_msgs/msg/AckermannDriveStamped "{drive: {speed: 0.5, steering_angle: 0.2}}" --rate 10

   ```

4. **Test emergency** — stop all inputs and wait `safety_threshold` seconds (0.5 s default). The output should switch to `speed: -0.5, steering_angle: 0.0`.

## Definition of Good Result

| Check | Expected |
|-------|----------|
| Output publish rate | 50 Hz (matches `publish_rate_hz`) |
| Output when joystick publishing | Matches joystick command exactly |
| Output when joystick masked (`is_idle = true`) | Drops to next priority input |
| Output when no inputs active | Emergency: `speed = -0.5, steering_angle = 0.0` |
| Safety trip (joystick silent > 0.5 s) | Emergency published within one publish cycle |
