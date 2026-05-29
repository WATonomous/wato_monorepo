# Developing pid_control

## Topics

### ackermann_smoother_node

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribed | `ackermann_in` | `ackermann_msgs/AckermannDriveStamped` | Raw setpoint from planner |
| Published | `ackermann_out` | `ackermann_msgs/AckermannDriveStamped` | Rate-limited setpoint |

### pid_control_node

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribed | `ackermann` | `ackermann_msgs/AckermannDriveStamped` | Desired steering angle and speed |
| Subscribed | `steering_feedback` | `roscco_msg/SteeringAngle` | Current wheel angle |
| Subscribed | `velocity_feedback` | `std_msgs/Float64` | Current velocity (m/s) |
| Published | `roscco` | `roscco_msg/Roscco` | Steering torque + throttle/brake command |

### vel_driven_feedforward_pid_node

| Direction | Topic | Type | Description |
|-----------|-------|------|-------------|
| Subscribed | `ackermann` | `ackermann_msgs/AckermannDriveStamped` | Desired steering angle and speed |
| Subscribed | `steering_feedback` | `roscco_msg/SteeringAngle` | Current wheel angle |
| Subscribed | `velocity_feedback` | `std_msgs/Float64` | Current velocity from CAN |
| Subscribed | `odom_feedback` | `nav_msgs/Odometry` | Current velocity from odometry (alternative) |
| Published | `roscco` | `roscco_msg/Roscco` | Final command (PID + feedforward) |
| Published | `feedforward` | `pid_msgs/Feedforward` | Feedforward component only (diagnostics) |
| Published | `velocity_derived` | `std_msgs/Float64` | EMA-filtered velocity used internally (diagnostics) |

## Parameters

### ackermann_smoother_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_steering_accel` | double | `2.0` | Max rate of change of steering angle (rad/s²) |
| `max_speed_accel` | double | `2.0` | Max rate of change of speed (m/s³) |
| `publish_rate` | double | `50.0` | Output publish rate (Hz) |

### pid_control_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_rate` | double | `20.0` | Control loop rate (Hz) |
| `steering_pid.p/i/d` | double | `2.5/0.0/0.1` | Steering PID gains |
| `steering_pid.i_clamp_max/min` | double | `1.0/−1.0` | Integrator saturation limits |
| `steering_pid.u_clamp_max/min` | double | `1.0/−1.0` | Output saturation limits |
| `steering_pid.antiwindup` | bool | `true` | Enable anti-windup |
| `steering_pid.antiwindup_strategy` | string | `conditional_integration` | Anti-windup method |
| `velocity_pid.p/i/d` | double | `1.0/0.1/0.01` | Velocity PID gains |

### vel_driven_feedforward_pid_node

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `update_rate` | double | `20.0` | Control loop rate (Hz) |
| `steering.output_clamp_max/min` | float | `1.0/−1.0` | Total steering output saturation |
| `steering.d_on_measurement` | double | `0.2` | D-on-measurement gain (avoids derivative kick on setpoint changes) |
| `steering.pid.p/i/d` | double | `4.0/0.2/0.0` | Steering PID gains |
| `steering.feedforward.coefficients` | double[3] | `[−0.220738, 0.156354, −0.003626]` | Feedforward polynomial coefficients `[c0, c1, c2]` |
| `steering.feedforward.friction_offset` | double | `0.079768` | Friction compensation offset |
| `velocity.filter_alpha` | double | `0.05` | EMA filter factor on velocity (0 = no update, 1 = no smoothing) |
| `velocity.pid.p/i/d` | double | `0.4/0.1/0.0` | Velocity PID gains |
| `velocity.output.throttle_scale` | double | `1.0` | Scale positive PID effort → throttle command |
| `velocity.output.brake_scale` | double | `1.0` | Scale negative PID effort → brake command |
| `velocity.output.deadband` | double | `0.05` | Effort below this magnitude → coast (forward = 0) |

## Build & Launch

```bash
colcon build --packages-select pid_control

# Launch individual nodes (typically via interfacing_can.launch.yaml)
ros2 launch pid_control vel_driven_feedforward_pid.launch.yaml
```

Config is loaded from `config/vel_driven_feedforward_pid.yaml` or via the `config_file` launch argument.

## After Launching

1. **Verify lifecycle activation** — look for this log line from the active node:

   ```
   Activated - control loop running at 20.0 Hz
   ```

   For `vel_driven_feedforward_pid_node`, also confirm the velocity source:

   ```
   Velocity source locked to CAN (Float64)
   ```

   or `Velocity source locked to Odometry` depending on which topic publishes first.

2. **Verify output is publishing:**

```bash
   ros2 topic hz /roscco   # expect 20 Hz (matches update_rate)
   ```

1. **Check for waiting warnings** — if any of these appear after the first few seconds, the corresponding feedback topic is not publishing:

   ```
Waiting for ackermann setpoint...
   Waiting for steering feedback...
   Waiting for velocity feedback...

   ```

4. **Command the smoother** (if in use) — publish a setpoint and confirm `ackermann_out` tracks it with rate limiting:

   ```bash
   ros2 topic pub /ackermann_in ackermann_msgs/msg/AckermannDriveStamped \
  "{drive: {speed: 1.0, steering_angle: 0.3}}" --rate 20
   ros2 topic echo /ackermann_out
   ```

## Definition of Good Result

| Check | Expected |
|-------|----------|
| Steady-state steering error | < 0.05 rad (≈ 3 deg) |
| Steady-state velocity error | < 0.1 m/s |
| No "Waiting for..." warnings after startup | All feedback topics are live |
| `feedforward` topic (FF node only) | Non-zero when vehicle is moving and steering is commanded |
| Smoother output at step input | Ramps to target within `max_steering_accel` / `max_speed_accel` limits — no instantaneous jumps |

## Internal Architecture

**Control loop:** All three nodes run a wall timer at `update_rate` Hz. The timer reads the latest cached setpoints and feedback values (stored in subscriber callbacks) and computes the output command. The loop does not block waiting for new messages — if a feedback topic is stale, the last received value is reused.

**Velocity source (vel_driven_feedforward_pid_node):** On first activation, the node subscribes to both `velocity_feedback` (Float64 from CAN) and `odom_feedback` (Odometry). Whichever topic publishes first becomes the velocity source for the session. The chosen source is logged at activation. The EMA filter (`velocity.filter_alpha`) smooths the selected velocity before it enters the PID loop — a lower alpha slows the filter response and reduces noise.

**Feedforward model:** The steering feedforward is a polynomial in vehicle speed: `T_ff = (c0 + c1·v + c2·v²) · steering_setpoint + friction_offset · sign(steering_setpoint)`. This compensates for increased tire scrub at higher speeds. Refit the coefficients with `analysis/` scripts whenever vehicle geometry or tire conditions change significantly.

**Smoother:** `ackermann_smoother_node` clips the per-tick rate of change of steering angle and speed, enforcing `max_steering_accel` and `max_speed_accel`. It is a pure rate limiter with no predictive model. Increase the limits if the vehicle is sluggish to respond; decrease them if actuators are commanded faster than they can physically follow.

## Fitting Feedforward Coefficients

The `analysis/` directory contains Python scripts to fit the polynomial feedforward model from bag data. See `analysis/README.md` for the full pipeline. After fitting, update the `steering.feedforward.coefficients` and `steering.feedforward.friction_offset` parameters in the config file.
