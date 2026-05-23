# Developing oscc_mux

## Topics

### Subscribed

Input topics are declared dynamically from the `inputs` parameter at configure time.

| Topic | Type | Description |
|-------|------|-------------|
| `inputs[*].topic` | `roscco_msg/Roscco` | Command from each configured source |
| `inputs[*].mask_topic` | `std_msgs/Bool` | Per-input mask; `true` = input is idle/disabled |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `roscco` | `roscco_msg/Roscco` | Selected command output |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `safety_threshold` | double | `0.5` | Max seconds before safety-gated input triggers emergency |
| `publish_rate_hz` | double | `50.0` | Output publish rate (Hz) |
| `emergency.steering` | float | `0.0` | Emergency steering torque |
| `emergency.forward` | float | `0.0` | Emergency throttle/brake (0 = coast) |
| `inputs.<name>.topic` | string | — | Input topic name |
| `inputs.<name>.priority` | int | — | Priority level; higher wins |
| `inputs.<name>.has_mask` | bool | `false` | Whether this input has a mask topic |
| `inputs.<name>.mask_topic` | string | `""` | Mask topic name |
| `inputs.<name>.safety_gating` | bool | `false` | Trigger emergency if this input goes stale |

## Example Configuration

```yaml
oscc_mux:
  ros__parameters:
    safety_threshold: 0.5
    publish_rate_hz: 50.0
    emergency:
      steering: 0.0
      forward: 0.0
    inputs:
      joystick:
        topic: /joystick/roscco
        priority: 100
        has_mask: true
        mask_topic: /joystick/is_idle
        safety_gating: false   # joystick may be silent when in Ackermann mode
      pid:
        topic: /pid/roscco
        priority: 10
        has_mask: false
        safety_gating: false
```

## Build & Launch

```bash
colcon build --packages-select oscc_mux
# Launched by interfacing_bringup/interfacing_can.launch.yaml
```
