# pid_control

The `pid_control` package provides a ROS 2 node that implements dual PID control loops for steering and velocity. It is designed to interface with the OSCC (Open Source Car Control) system by outputting combined control commands.

## Overview

The `pid_control_node` subscribes to desired vehicle states (steering angle and speed) and real-time feedback from sensors. It computes error signals for both steering and velocity independently, applies PID control logic with anti-windup, and publishes a `Roscco` message containing the required steering torque and velocity commands.

## Node: `pid_control_node`

### Subscriptions

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/joystick/ackermann` | `ackermann_msgs/msg/AckermannDriveStamped` | Desired steering angle and speed setpoints. |
| `/steering_meas` | `std_msgs/msg/Float64` | Current steering angle feedback. |
| `/velocity_meas` | `std_msgs/msg/Float64` | Current vehicle velocity feedback. |

### Publishers

| Topic | Message Type | Description |
| :--- | :--- | :--- |
| `/joystick/roscco` | `roscco_msg/msg/Roscco` | Combined control output (steering torque and forward command). |

### Parameters

#### General
- `ackermann_topic` (string, default: `/joystick/ackermann`): Topic for desired setpoints.
- `steering_feedback_topic` (string, default: `/steering_meas`): Topic for steering feedback.
- `velocity_feedback_topic` (string, default: `/velocity_meas`): Topic for velocity feedback.
- `roscco_topic` (string, default: `/joystick/roscco`): Output topic for OSCC commands.
- `update_rate` (double, default: `100.0`): Frequency (Hz) of the PID control loop.

#### PID Configuration (`steering_pid` and `velocity_pid` namespaces)
Both steering and velocity controllers share the same parameter structure:

- `p` (double): Proportional gain.
- `i` (double): Integral gain.
- `d` (double): Derivative gain.
- `i_clamp_max` (double): Upper bound for the integrator term (anti-windup).
- `i_clamp_min` (double): Lower bound for the integrator term.
- `u_clamp_max` (double): Upper saturation limit for the controller output.
- `u_clamp_min` (double): Lower saturation limit for the controller output.
- `saturation` (bool): Whether to enforce output saturation limits.
- `antiwindup` (bool): Enable anti-windup/integrator correction.
- `antiwindup_strategy` (string): Strategy for anti-windup (recommended: `conditional_integration`).

## Usage

### Launching the node
The node can be launched using the provided launch file, which automatically loads the configuration from `pid_control.yaml`:

```bash
ros2 launch pid_control pid_control.launch.yaml
```

## Anti-Windup Strategy
This package uses the `conditional_integration` strategy by default. This is a modern anti-windup method provided by `control_toolbox` that silences legacy deprecation warnings while providing robust integrator behavior during actuator saturation.
