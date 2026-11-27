# Developing can_state_estimator

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `can_state_estimator/steering_angle` | `roscco_msg/SteeringAngle` | Current wheel angle (radians) |
| `can_state_estimator/body_velocity` | `std_msgs/Float64` | Rear-axle longitudinal velocity (m/s) |
| `can_state_estimator/wheel_joint_states` | `sensor_msgs/JointState` | Per-wheel joint velocities (rad/s, unsigned) for wheel odometry consumers |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `can_interface` | string | `can1` | SocketCAN interface for the vehicle OBD bus |
| `steering_conversion_factor` | double | `15.7` | Steering wheel to wheel angle ratio |
| `wheel_radius` | double | `0.31235` | Wheel radius (m) used to convert wheel speed to joint velocity |
| `front_left_joint` | string | `front_left_wheel_joint` | Joint name for the front-left wheel |
| `front_right_joint` | string | `front_right_wheel_joint` | Joint name for the front-right wheel |
| `rear_left_joint` | string | `rear_left_joint` | Joint name for the rear-left wheel |
| `rear_right_joint` | string | `rear_right_joint` | Joint name for the rear-right wheel |

## Constants

| Constant | Value | Description |
|----------|-------|-------------|
| `STEERING_ANGLE_CAN_ID` | `0x2B0` | CAN ID for steering wheel angle frames |
| `WHEEL_SPEED_CAN_ID` | `0x4B0` | CAN ID for wheel speed frames |
| `STEERING_ANGLE_SCALAR` | `0.1` | Degrees per bit in the steering frame |
| `KPH_TO_MPS` | `1.0 / 3.6` | Conversion factor from km/h to m/s |

## CAN Frame Decoding

**Steering angle (0x2B0):**
- Bytes 0–1: int16 little-endian, scaled at `STEERING_ANGLE_SCALAR` deg/bit
- Formula: `wheel_angle_rad = -raw * 0.1 * (π/180) / steering_conversion_factor`

**Wheel speeds (0x4B0):**
- Four 12-bit values at byte offsets 0, 2, 4, 6
- Masking: `raw = ((data[offset+1] & 0x0F) << 8) | data[offset]`
- Convert: `speed_kmh = (int)(raw / 3.2) / 10.0`
- Layout: NW (left-front), NE (right-front), SW (left-rear), SE (right-rear)

## Build & Launch

```bash
colcon build --packages-select can_state_estimator
ros2 launch can_state_estimator can_state_estimator.launch.yaml
```

## Internal Architecture

**Threading:** A background thread blocks on `read()` for CAN frames. The ROS executor and publishers are on the main thread. A single mutex protects the shared steering angle and wheel speed values — the lock is held only long enough to copy doubles, so contention is negligible.

**Lifecycle callbacks:**
- `on_configure`: Opens SocketCAN socket, sets kernel-level filter for IDs `0x2B0` and `0x4B0`.
- `on_activate`: Starts CAN read thread.
- `on_deactivate`: Signals and joins CAN read thread.
- `on_cleanup`: Closes socket, destroys publishers.

**Socket filter:** The kernel-level CAN filter is set in `on_configure` to pass only IDs `0x2B0` and `0x4B0`. All other frames are rejected in the kernel before reaching userspace, so the read thread only wakes on relevant frames. To add a new CAN signal, register its ID in the filter array in `on_configure`.

## Design Rationale

CAN frames are read directly via SocketCAN rather than subscribing to OSCC topics because:
1. Eliminates the dependency on `oscc_interfacing` for feedback data.
2. Lower latency — no extra ROS hop between CAN and state estimation.
3. A single node handles both steering and wheel speed, keeping state consistent.

This node does not integrate odometry. Wheel speeds are published as `sensor_msgs/JointState` (joint names match the `eve_description` URDF) and wheel odometry is computed by `eidos::WheelOdomFactor`, which fuses rear wheel speed with IMU yaw rate.

## After Launching

1. **Verify lifecycle transition** — the node is managed by `wato_lifecycle_manager`. Check it reaches active state:

   ```bash
   ros2 lifecycle get /can_state_estimator_node   # expect: active
   ```

2. **Verify topics are publishing:**

```bash
   ros2 topic hz /can_state_estimator/steering_angle   # publishes on each 0x2B0 frame (~50–100 Hz)
   ros2 topic hz /can_state_estimator/body_velocity    # publishes on each 0x4B0 frame
   ros2 topic hz /can_state_estimator/wheel_joint_states   # publishes on each 0x4B0 frame
   ```

1. **Sanity-check steering angle** — turn the steering wheel to full lock and echo the topic:

   ```bash
ros2 topic echo /can_state_estimator/steering_angle --once

   ```

4. **Sanity-check velocity** — drive at a known speed and compare:

   ```bash
   ros2 topic echo /can_state_estimator/body_velocity --once
```

## Definition of Good Result

| Check | Expected |
|-------|----------|
| Steering angle at centre | Within ±0.03 rad of 0.0 |
| Steering angle at full lock | Matches physical limit (typically ±0.55 rad) |
| Body velocity at 10 km/h | Within ±0.2 m/s of 2.78 m/s |
| No CAN errors in log | No `"Failed to read CAN frame"` or socket error messages |

If topics are not publishing, common causes:
- Wrong `can_interface` parameter (check with `ip link show`)
- CAN socket not up (`sudo ip link set can1 up type can bitrate 500000`)

## Adding New CAN Signals

1. Add the CAN ID constant and add it to the kernel filter array in `on_configure`.
2. Add a `process_*_frame()` method with the decoding logic.
3. Add the dispatch case in `read_loop()`.
4. Add any new publishers as lifecycle publishers; activate/deactivate in the lifecycle callbacks.
