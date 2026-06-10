# roscco_msg

Custom ROS message types for the ROSCCO (Open Source Car Control) interface between the PID controller, joystick, and OSCC hardware node.

## Messages

### Roscco.msg

Combined steering and throttle/brake command sent to `oscc_interfacing`.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `steering` | `float32` | Steering torque (−1.0 to 1.0) |
| `forward` | `float32` | Throttle (positive) or brake (negative), −1.0 to 1.0 |

### WheelSpeeds.msg

Four individual wheel speeds published by `oscc_interfacing` from OBD CAN frames.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `ne` | `float32` | Right-front wheel speed (km/h) |
| `nw` | `float32` | Left-front wheel speed (km/h) |
| `se` | `float32` | Right-rear wheel speed (km/h) |
| `sw` | `float32` | Left-rear wheel speed (km/h) |

Cardinal names (NE/NW/SE/SW) follow the vehicle compass convention used throughout the codebase (N = front, S = rear, E = right, W = left).

### SteeringAngle.msg

Wheel steering angle in radians, published by both `oscc_interfacing` and `can_state_estimator`.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `angle` | `float32` | Front wheel angle (radians); 0 = straight ahead |

### SteeringTorque.msg

Raw steering torque feedback from the OSCC steering module.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `torque` | `float32` | Measured steering column torque |
