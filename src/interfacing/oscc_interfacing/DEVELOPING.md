# Developing oscc_interfacing

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `roscco` | `roscco_msg/Roscco` | Steering torque + throttle/brake commands |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `oscc_interfacing/is_armed` | `std_msgs/Bool` | Current arming state, published at `is_armed_publish_rate_hz` |
| `oscc_interfacing/wheel_speeds` | `roscco_msg/WheelSpeeds` | Four wheel speeds (NE, NW, SE, SW) in km/h |
| `oscc_interfacing/steering_angle` | `roscco_msg/SteeringAngle` | Steering wheel angle feedback |

### Services (Server)

| Service | Type | Description |
|---------|------|-------------|
| `/oscc_interfacing/arm` | `std_srvs/SetBool` | `true` = arm, `false` = disarm. Returns success/failure. |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `is_armed_publish_rate_hz` | int | `100` | Rate to publish arming status (Hz) |
| `oscc_can_bus` | int | `0` | CAN bus index (0 or 1) passed to the OSCC library |
| `steering_scaling` | double | `1.0` | Scale factor (0–1) applied to steering commands |
| `steering_torque_deadzone_pos` | double | `0.09` | Deadzone offset added to positive steering torque |
| `steering_torque_deadzone_neg` | double | `0.13` | Deadzone offset added to negative steering torque |
| `steering_conversion_factor` | double | `15.7` | Steering wheel to wheel angle ratio (for angle feedback) |
| `disable_boards_on_fault` | bool | `false` | If `true`, disable individual OSCC modules on fault; if `false`, disarm only |
| `enable_all` | bool | `true` | Arm/disarm all modules together |
| `enable_steering` | bool | `true` | Enable steering module on arm |
| `enable_throttle` | bool | `true` | Enable throttle module on arm |
| `enable_brakes` | bool | `true` | Enable brake module on arm |

## Build & Launch

```bash
colcon build --packages-select oscc_interfacing
ros2 launch oscc_interfacing oscc_interfacing.launch.yaml
```

## Internal Architecture

The OSCC C library runs its own CAN communication thread. This node registers callbacks with the library and caches their outputs:

- `brake_report_callback` / `throttle_report_callback` — detect operator override; trigger disarm
- `steering_report_callback` — steering torque feedback + override detection
- `obd_callback` — decode wheel speeds (four values) and steering wheel angle from OBD frames
- `fault_report_callback` — identify which module faulted; disarm or disable accordingly

Callback data is written under a mutex or via atomics. The ROS subscriber callback and the publish timer read from the same cached state.

**Steering deadzone:** The Kia Soul EV OSCC steering module has a mechanical deadzone near zero torque. The `steering_torque_deadzone_pos/neg` offsets compensate for this by shifting the command magnitude so small inputs still produce motion.

## After Launching

1. **Arm the vehicle:**

   ```bash
   ros2 service call /oscc_interfacing/arm std_srvs/srv/SetBool "{data: true}"
   # Expect: success=True
   ```

2. **Verify arming state:**

```bash
   ros2 topic echo /oscc_interfacing/is_armed --once
   # Expect: data: true
   ```

1. **Verify feedback topics are publishing:**

   ```bash
ros2 topic hz /oscc_interfacing/wheel_speeds     # publishes at CAN OBD rate
   ros2 topic hz /oscc_interfacing/steering_angle   # publishes at CAN OBD rate

   ```

4. **Disarm when done:**

   ```bash
   ros2 service call /oscc_interfacing/arm std_srvs/srv/SetBool "{data: false}"
```

## Definition of Good Result

| Check | Expected |
|-------|----------|
| Arm service response | `success=True` with no error message |
| `is_armed` after arming | `true` at 100 Hz |
| `wheel_speeds` fields | Non-zero and changing when wheels are spinning |
| `steering_angle` | Changes smoothly when steering wheel is turned |
| No fault callbacks | No `"override detected"` or fault messages in node log |
| Disarm via button | `is_armed` drops to `false`, long joystick vibration pulse |

If arm fails, common causes:
- CAN bus not up (`oscc_can_bus` parameter pointing to wrong bus)
- OSCC boards not powered
- `disable_boards_on_fault` triggered by a prior fault — power-cycle the OSCC boards
