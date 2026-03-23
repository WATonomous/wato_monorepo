# pid_control

The `pid_control` package provides ROS 2 lifecycle nodes for low-level vehicle control. It outputs `Roscco` commands for the OSCC (Open Source Car Control) system.

## Nodes

### 1. `pid_control_node`

Basic dual PID controller for steering and velocity. Subscribes to an Ackermann setpoint and sensor feedback, runs independent PID loops, and publishes a combined `Roscco` command.

| Subscriptions | Type | Description |
|:---|:---|:---|
| `ackermann` | `ackermann_msgs/AckermannDriveStamped` | Desired steering angle and speed |
| `steering_feedback` | `roscco_msg/SteeringAngle` | Current steering angle |
| `velocity_feedback` | `std_msgs/Float64` | Current vehicle velocity |

| Publishers | Type | Description |
|:---|:---|:---|
| `roscco` | `roscco_msg/Roscco` | Steering torque + velocity command |

### 2. `vel_driven_feedforward_pid_node`

Feedforward + PID controller for steering and velocity. Adds a velocity-dependent feedforward torque model on top of the steering PID to improve tracking at speed. Also supports D-on-measurement to avoid derivative kick, an EMA filter on velocity, and configurable throttle/brake scaling.

The feedforward model is:

```
T_ff = (c0 + c1*v + c2*v²) * steering_setpoint + friction_offset * sign(steering_setpoint)
```

Coefficients are fit from bag data using the scripts in `analysis/` (see `analysis/README.md`). Parameters are hot-reloadable at runtime.

| Subscriptions | Type | Description |
|:---|:---|:---|
| `ackermann` | `ackermann_msgs/AckermannDriveStamped` | Desired steering angle and speed |
| `steering_feedback` | `roscco_msg/SteeringAngle` | Current steering angle |
| `velocity_feedback` | `std_msgs/Float64` | Current velocity (CAN) |
| `odom_feedback` | `nav_msgs/Odometry` | Current velocity (odometry, alternative source) |

| Publishers | Type | Description |
|:---|:---|:---|
| `roscco` | `roscco_msg/Roscco` | Steering torque + velocity command |
| `feedforward` | `pid_msgs/Feedforward` | Feedforward debug output |
| `velocity_derived` | `std_msgs/Float64` | Filtered velocity used internally |

### 3. `ackermann_smoother_node`

Rate-limiter that sits between the planner and PID. Smooths incoming Ackermann setpoints by limiting steering and speed acceleration, preventing abrupt jumps that could destabilize control or cause mechanical stress.

| Subscriptions | Type | Description |
|:---|:---|:---|
| `ackermann_in` | `ackermann_msgs/AckermannDriveStamped` | Raw setpoint from planner |

| Publishers | Type | Description |
|:---|:---|:---|
| `ackermann_out` | `ackermann_msgs/AckermannDriveStamped` | Smoothed setpoint for PID |

## Configuration

- `config/vel_driven_feedforward_pid.yaml` — parameters for the feedforward PID node
- Config is loaded via the launch file's `config_file` argument

## Analysis Tools

The `analysis/` directory contains Python scripts for fitting the feedforward steering torque model from bag data. See `analysis/README.md` for the full pipeline.
