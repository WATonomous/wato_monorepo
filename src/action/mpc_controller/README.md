# MPC Controller

ROS2 lifecycle node that tracks a trajectory using Model Predictive Control (MPC), publishing Ackermann drive commands for vehicle steering and speed control. Drop-in replacement for `ackermann_pure_pursuit` — same lifecycle, idle/standby conventions, and Ackermann output.

## Overview

Each control cycle, the node reads the vehicle's pose from TF and speed from odometry, resamples the planned trajectory into a fixed-spacing reference, and solves a quadratic program (QP) that finds the steering and acceleration sequence best tracking that reference over a short horizon. Only the first command of the optimal sequence is applied; the rest is discarded and the problem is re-solved on the next cycle (receding horizon). It publishes an idle signal and a standby Ackermann command when no valid trajectory is available, the trajectory has gone stale, or the behaviour tree requests standby.

Unlike a geometric tracker (e.g. pure pursuit), MPC plans against a vehicle model and explicit limits, so it can slow down *before* a curve, respect acceleration/jerk limits for ride comfort, and trade off competing objectives (tracking accuracy vs. smoothness vs. progress) through tunable cost weights.

**Current Status**: Functional linear time-varying MPC using a slip-corrected kinematic bicycle model and the OSQP solver (via `osqp_eigen_vendor`). Supports curvature/speed feedforward, warm-starting, optional latency compensation, and predicted-path visualization.

## Algorithm

The controller is a **linear time-varying MPC (LTV-MPC)**: the nonlinear vehicle model is linearized at each point along the horizon, producing a sequence of linear models, and the resulting convex QP is solved with OSQP.

### Vehicle model

Motion is predicted with a slip-corrected **kinematic bicycle model** (shared `vehicle_models` package). State is `[x, y, θ, v]` (position, heading, speed); control is `[δ, a]` (steering angle, longitudinal acceleration). The body slip angle `β = atan((lr/L)·tan(δ))` references the model at the centre of gravity. Tire forces are not modelled — valid in the normal (non-limit-handling) regime. Continuous dynamics are integrated with RK4 and discretized.

### Control pipeline

Each cycle (`control_rate_hz`, default 20 Hz):

1. **Sample reference** — find the nearest trajectory point, walk forward at fixed arc-length spacing (`point_spacing`) up to `horizon_distance` / `max_horizon_steps`. Reference headings are unwrapped to be continuous with the current heading (avoids ±π wrap injecting spurious error), per-point `dt` is derived from the reference speed, and path curvature drives a steering feedforward (`δ_ref = atan(L·κ)`) used as the linearization operating point.
2. **Linearize** the bicycle model at each reference point about its feedforward control, yielding an affine model `x_{k+1} = A_k·x_k + B_k·u_k + g_k` per step.
3. **Solve the QP** for the optimal state/control sequence.
4. **Apply** the first control `u_0` as the Ackermann command; cache it for next cycle's rate constraints.

### QP cost

The optimizer minimizes a weighted sum of:

- **Position tracking** in the path frame — cross-track (perpendicular) error weighted by `w_lateral` and along-track error by `w_long`, so tracking is independent of road orientation.
- **Heading tracking** (`w_heading`).
- **Progress** — a linear speed reward (`w_progress`) and/or quadratic speed tracking toward the reference profile (`w_speed`).
- **Control effort** (`w_steering`, `w_accel`) and **control rate** (`w_dsteering` for steering smoothness, `w_daccel` for jerk comfort).
- A **terminal multiplier** (`w_terminal`) on the final-step tracking weights to stabilize the horizon tail.

### QP constraints (hard)

- Initial state pinned to the measured (optionally latency-compensated) state.
- Dynamics equality at every step (predicted states must obey the linearized model).
- Speed bounds (`0 … min(reference limit, max_speed)`), actuator bounds (`max_steering_angle`, `max_accel`, `max_decel`), and rate bounds (`max_steering_rate`, `max_jerk`).

The solver is reused across cycles (in-place matrix updates) and warm-started from the previous solution time-shifted forward; it fully re-initializes only when the horizon length changes. On solver failure the node holds the previous steering and coasts.

## Dependencies

- ROS 2 (tested on Humble)
- `osqp_eigen_vendor` / `OsqpEigen`, `Eigen3` — QP solver
- `vehicle_models` — shared kinematic bicycle model
- `tf2_ros`, `tf2_geometry_msgs` — vehicle pose lookup
- `ackermann_msgs`, `nav_msgs`, `std_msgs`, `geometry_msgs`
- `wato_trajectory_msgs` (custom trajectory message)
- `behaviour_msgs` (custom behaviour message)
- `rclcpp_lifecycle`, `rclcpp_components`, `wato_lifecycle_manager`

## Further Reading

See [docs/DEVELOPING.md](docs/DEVELOPING.md) for the full ROS interface, parameter reference, build/launch instructions, verification checklist, and internal architecture.

## License

Copyright (c) 2025-present WATonomous. All rights reserved.
Licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).
