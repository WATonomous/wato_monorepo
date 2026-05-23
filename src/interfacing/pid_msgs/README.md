# pid_msgs

Custom ROS message types for PID controller diagnostics in the `pid_control` package.

## Messages

### PidGains.msg

Snapshot of PID tuning gains, used for live diagnostics and gain logging.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `p` | `float64` | Proportional gain |
| `i` | `float64` | Integral gain |
| `d` | `float64` | Derivative gain |

### Feedforward.msg

Feedforward torque contribution from the velocity-dependent steering model, published by `vel_driven_feedforward_pid_node` for diagnostics.

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Timestamp |
| `feedforward` | `float64` | Computed feedforward torque value |
