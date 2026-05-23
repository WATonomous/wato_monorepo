# can_state_estimator

Reads steering angle and wheel speed frames directly from the vehicle OBD CAN bus and publishes steering angle, body velocity, and dead-reckoning odometry.

## Overview

Rather than routing vehicle feedback through OSCC, this node reads CAN frames directly via SocketCAN. This eliminates the OSCC dependency for feedback data and reduces latency by one ROS hop. It handles both the `0x2B0` (steering) and `0x4B0` (wheel speeds) CAN frame IDs used by the Kia Soul EV.

## Architecture

The node runs a background thread that blocks on `read()` for incoming CAN frames. Each frame is decoded and the corresponding state (steering angle, wheel speeds) is updated under a mutex. On each wheel speed frame the node recomputes body velocity and integrates odometry.

```
CAN Bus (SocketCAN)
  0x2B0 steering ──┐
  0x4B0 wheels  ──┤──► CAN read thread ──► decode & integrate ──► publishers
                   │
              TF lookup (rear_axle → front_axle = wheelbase)
```

**Ackermann bicycle model** (rear-axle reference):
```
v_front_avg = (v_nw + v_ne) / 2
v_body      = v_front_avg * cos(steering_angle)
omega       = v_body * tan(steering_angle) / wheelbase

x     += v_body * cos(theta) * dt
y     += v_body * sin(theta) * dt
theta += omega * dt
```

The wheelbase is resolved from TF at startup by looking up the distance between `rear_axle_frame` and `front_axle_frame` (published by `robot_state_publisher` from the URDF). The node blocks until this transform is available before publishing velocity or odometry.

Odometry is pure dead-reckoning and will drift. Fuse with GPS/IMU for absolute positioning.

## Lifecycle

Managed by `wato_lifecycle_manager`:

| Transition | Action |
|------------|--------|
| configure | Read parameters, create publishers, open and bind CAN socket, start TF listener |
| activate | Activate publishers, reset odometry, start CAN read thread |
| deactivate | Stop CAN read thread, deactivate publishers |
| cleanup | Close CAN socket, destroy publishers and TF resources |
