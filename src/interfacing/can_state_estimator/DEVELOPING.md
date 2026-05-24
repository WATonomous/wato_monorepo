# Developing can_state_estimator

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `can_state_estimator/steering_angle` | `roscco_msg/SteeringAngle` | Current wheel angle (radians) |
| `can_state_estimator/body_velocity` | `std_msgs/Float64` | Rear-axle longitudinal velocity (m/s) |
| `can_state_estimator/odom` | `nav_msgs/Odometry` | Dead-reckoning pose and twist |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `can_interface` | string | `can1` | SocketCAN interface for the vehicle OBD bus |
| `steering_conversion_factor` | double | `15.7` | Steering wheel to wheel angle ratio |
| `rear_axle_frame` | string | `rear_axle` | TF frame at the rear axle (wheelbase source) |
| `front_axle_frame` | string | `front_axle` | TF frame at the front axle (wheelbase target) |
| `odom_frame` | string | `odom` | Frame ID for the odometry header |
| `base_frame` | string | `base_footprint` | Child frame ID for the odometry message |

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

Odometry state (`x`, `y`, `theta`, `last_time`) is only accessed from the CAN read thread and therefore does not need the mutex.

**Lifecycle callbacks:**
- `on_configure`: Opens SocketCAN socket, sets kernel-level filter for IDs `0x2B0` and `0x4B0`, initialises TF listener.
- `on_activate`: Starts CAN read thread, resets odometry to zero.
- `on_deactivate`: Signals and joins CAN read thread.
- `on_cleanup`: Closes socket, destroys publishers.

## Design Rationale

CAN frames are read directly via SocketCAN rather than subscribing to OSCC topics because:
1. Eliminates the dependency on `oscc_interfacing` for feedback data.
2. Lower latency — no extra ROS hop between CAN and state estimation.
3. A single node handles both steering and wheel speed, keeping state consistent.

The wheelbase is looked up from TF (published by `robot_state_publisher` from `eve_description`) instead of being hardcoded so vehicle geometry is defined in one place (the URDF).

## After Launching

1. **Verify lifecycle transition** — the node is managed by `wato_lifecycle_manager`. Check it reaches active state:
   ```bash
   ros2 lifecycle get /can_state_estimator_node   # expect: active
   ```

2. **Verify topics are publishing:**
   ```bash
   ros2 topic hz /can_state_estimator/steering_angle   # publishes on each 0x2B0 frame (~50–100 Hz)
   ros2 topic hz /can_state_estimator/body_velocity    # publishes on each 0x4B0 frame
   ros2 topic hz /can_state_estimator/odom
   ```

3. **Sanity-check steering angle** — turn the steering wheel to full lock and echo the topic:
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
| Odometry drift over 100 m straight | < 2 m lateral (dead-reckoning only) |
| No CAN errors in log | No `"Failed to read CAN frame"` or socket error messages |

If topics are not publishing, common causes:
- Wrong `can_interface` parameter (check with `ip link show`)
- CAN socket not up (`sudo ip link set can1 up type can bitrate 500000`)
- TF wheelbase lookup pending — wait for `eve_description` TF to publish

## Adding New CAN Signals

1. Add the CAN ID constant and add it to the kernel filter array in `on_configure`.
2. Add a `process_*_frame()` method with the decoding logic.
3. Add the dispatch case in `read_loop()`.
4. Add any new publishers as lifecycle publishers; activate/deactivate in the lifecycle callbacks.
