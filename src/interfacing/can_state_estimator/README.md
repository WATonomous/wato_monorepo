# can_state_estimator

Vehicle state estimation from the CAN bus.

This node reads steering angle and wheel speed frames directly from SocketCAN and publishes steering angle, body velocity, and dead-reckoning odometry. It replaces the separate `car_steering_feedback` and `car_velocity_feedback` packages with a single lifecycle node.

## Node

### can_state_estimator_node

Reads two CAN frame types from the vehicle OBD bus:

- **0x2B0** - Steering wheel angle (int16 LE, 0.1 deg/bit). Converted to wheel angle in radians using the `steering_conversion_factor`.
- **0x4B0** - Wheel speeds (four 12-bit values at 2-byte offsets, decoded as `(int)(raw / 3.2) / 10.0` in km/h).

Body velocity is computed as the average front wheel speed projected through the steering angle (Ackermann bicycle model). Odometry integrates this velocity and yaw rate over time.

The wheelbase is resolved from TF by looking up the distance between the `rear_axle_frame` and `front_axle_frame` (published by `robot_state_publisher` from the URDF). The node will wait for this transform before publishing velocity or odometry.

```bash
ros2 run can_state_estimator can_state_estimator_node
```

**Publications:**

| Topic | Type | Description |
|-------|------|-------------|
| `can_state_estimator/steering_angle` | `roscco_msg/SteeringAngle` | Wheel angle in radians |
| `can_state_estimator/body_velocity` | `std_msgs/Float64` | Rear-axle longitudinal velocity in m/s |
| `can_state_estimator/odom` | `nav_msgs/Odometry` | Dead-reckoning pose and twist |

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `can_interface` | string | `can1` | SocketCAN interface for the vehicle OBD bus |
| `steering_conversion_factor` | double | `15.7` | Steering wheel to wheel angle ratio |
| `rear_axle_frame` | string | `rear_axle` | TF frame at the rear axle (wheelbase source) |
| `front_axle_frame` | string | `front_axle` | TF frame at the front axle (wheelbase target) |
| `odom_frame` | string | `odom` | Frame ID for the odometry header |
| `base_frame` | string | `base_footprint` | Child frame ID for the odometry message |

## Lifecycle

The node is managed by `wato_lifecycle_manager`:

| Transition | Action |
|------------|--------|
| configure | Read parameters, create publishers, open and bind CAN socket, start TF listener |
| activate | Activate publishers, reset odometry, start CAN read thread |
| deactivate | Stop CAN read thread, deactivate publishers |
| cleanup | Close CAN socket, destroy publishers and TF resources |

## Odometry Model

Uses the Ackermann bicycle model referenced at the rear axle:

```
v_front_avg = (v_nw + v_ne) / 2          front wheel average (km/h -> m/s)
v_body      = v_front_avg * cos(delta)    longitudinal velocity at rear axle
omega       = v_body * tan(delta) / L     yaw rate (L = wheelbase from TF)

x     += v_body * cos(theta) * dt
y     += v_body * sin(theta) * dt
theta += omega * dt
```

This is pure dead-reckoning and will drift over time. Fuse with GPS/IMU for absolute positioning.
