# can_state_estimator

Reads steering angle and wheel speed frames directly from the vehicle OBD CAN bus and publishes steering angle, body velocity, and per-wheel joint velocities.

## Overview

Rather than routing vehicle feedback through OSCC, this node reads CAN frames directly via SocketCAN. This eliminates the OSCC dependency for feedback data and reduces latency by one ROS hop. It handles both the `0x2B0` (steering) and `0x4B0` (wheel speeds) CAN frame IDs used by the Kia Soul EV.

## Architecture

The node runs a background thread that blocks on `read()` for incoming CAN frames. Each frame is decoded and the corresponding state (steering angle, wheel speeds) is updated under a mutex. On each wheel speed frame the node publishes wheel joint states and recomputes body velocity.

```
CAN Bus (SocketCAN)
  0x2B0 steering ──┐
  0x4B0 wheels  ──┴──► CAN read thread ──► decode ──► publishers
```

**Body velocity** (PID feedback, rear-axle reference):

```
v_front_avg = (v_nw + v_ne) / 2
v_body      = v_front_avg * cos(steering_angle)
```

**Wheel joint states** (`can_state_estimator/wheel_joint_states`): each wheel speed is converted to a joint angular velocity `speed_mps / wheel_radius` (rad/s), using the URDF joint names. CAN wheel speeds are unsigned, so velocities are always ≥ 0 (reverse reads as forward).

This node does not integrate odometry. Wheel odometry is computed by `eidos::WheelOdomFactor` (see `src/world_modeling/eidos/docs/plugins/factors/wheel_odom_factor.md`).

## Lifecycle

Managed by `wato_lifecycle_manager`:

| Transition | Action |
|------------|--------|
| configure | Read parameters, create publishers, open and bind CAN socket |
| activate | Activate publishers, start CAN read thread |
| deactivate | Stop CAN read thread, deactivate publishers |
| cleanup | Close CAN socket, destroy publishers |
