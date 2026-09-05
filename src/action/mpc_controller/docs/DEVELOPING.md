# Developing mpc_controller

## Topics

Topic names below are the node-relative names (defaults); the deployed names are the remappings applied in `action_bringup/launch/action_mpc.launch.yaml` (node namespace `action`).

| Direction | Topic | Type | Deployed name | Description |
|-----------|-------|------|---------------|-------------|
| Subscribed | `trajectory` | `wato_trajectory_msgs/Trajectory` | `/action/trajectory_planning/trajectory` | Velocity-profiled trajectory from the trajectory planner |
| Subscribed | `odom` | `nav_msgs/Odometry` | `/world_modeling/transform/odometry` | Source of current longitudinal speed (`twist.twist.linear.x`) |
| Subscribed | `execute_behaviour` | `behaviour_msgs/ExecuteBehaviour` | `/behaviour/execute_behaviour` | Requested behaviour from the behaviour tree |
| Published | `ackermann` | `ackermann_msgs/AckermannDriveStamped` | `/action/ackermann` | Steering angle + speed command |
| Published | `is_idle` | `std_msgs/Bool` | `/action/is_idle` | `true` when idle or in standby |
| Published | `predicted_path` | `nav_msgs/Path` | `/action/mpc/predicted_path` | MPC's predicted state trajectory (visualization) |

Vehicle pose (`x, y, θ`) is obtained from a **TF lookup** of `map → base_frame` (not a topic). Speed comes from `odom`; together they form the state `[x, y, θ, v]`.

## Parameters

Defaults live in `config/mpc_controller.yaml` (loaded first), with real-vehicle overrides layered on top from `action_bringup/config/action.yaml`. The package YAML carries inline tuning ranges and guidance for every weight and limit.

### Topics & frames

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory_topic` | string | `trajectory` | Input trajectory topic |
| `bt_topic` | string | `execute_behaviour` | Behaviour-tree command topic |
| `ackermann_topic` | string | `ackermann` | Output Ackermann topic |
| `idle_topic` | string | `is_idle` | Idle-flag topic |
| `predicted_path_topic` | string | `predicted_path` | Predicted-path visualization topic |
| `odom_topic` | string | `odom` | Odometry topic (speed source) |
| `base_frame` | string | `base_footprint` | Vehicle base frame for the `map → base_frame` pose lookup |

### Control & standby

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `control_rate_hz` | double | `20.0` | Control loop / publish rate (Hz) |
| `idle_timeout_sec` | double | `2.0` | Time since last trajectory before declaring idle (s) |
| `standby_speed` | double | `0.0` | Speed commanded in standby (m/s) |
| `standby_steering` | double | `0.0` | Steering commanded in standby (rad) |
| `standby_msg` | string | `standby` | Behaviour string that triggers standby |
| `invert_steering` | bool | `false` | Invert the output steering sign (platform-dependent) |
| `disable_standby` | bool | `false` | If `true`, bypass standby and run MPC even on `standby_msg` |

### Vehicle model

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `wheelbase` | double | `2.5667` | Front-to-rear axle distance (m) |
| `lr` | double | `0.0` | Rear-axle→CoG distance for slip-angle correction (m); `<=0` defaults to `wheelbase/2` |

### Horizon

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `horizon_distance` | double | `25.0` | How far ahead MPC looks (m) |
| `point_spacing` | double | `1.0` | Arc-length between reference points (m) |
| `max_horizon_steps` | int | `30` | Cap on QP horizon steps (caps problem size) |

### Cost weights

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `w_lateral` | double | `50.0` | Cross-track (perpendicular) error weight |
| `w_long` | double | `5.0` | Along-track (longitudinal) position error weight |
| `w_heading` | double | `20.0` | Heading error weight |
| `w_progress` | double | `5.0` | Linear speed reward (`0` disables) |
| `w_speed` | double | `0.0` | Quadratic speed tracking toward reference speed (`0` disables; uses `w_progress` instead) |
| `w_steering` | double | `1.0` | Steering effort penalty |
| `w_accel` | double | `1.0` | Acceleration effort penalty |
| `w_dsteering` | double | `100.0` | Steering rate penalty — primary steering-comfort knob |
| `w_daccel` | double | `50.0` | Jerk penalty — primary longitudinal-comfort knob |
| `w_terminal` | double | `5.0` | Multiplier on tracking weights at the final horizon step |

### Actuator limits (hard)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_steering_angle` | double | `0.5` | Max steering angle magnitude (rad) |
| `max_accel` | double | `2.5` | Max longitudinal acceleration (m/s²) |
| `max_decel` | double | `-4.0` | Max deceleration, negative (m/s²) |
| `max_speed` | double | `15.0` | Global speed cap on top of planner limits (m/s) |

### Comfort limits (hard rate constraints)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_steering_rate` | double | `0.3` | Max steering rate (rad/s) |
| `max_jerk` | double | `5.0` | Max jerk (m/s³) |

### Solver

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `dt_max` | double | `0.5` | Cap on per-step `dt`, hit when speed is near zero (s) |
| `latency_sec` | double | `0.0` | Actuation+compute delay to compensate by rolling the initial state forward (`0` = off) |
| `max_solver_iterations` | int | `200` | OSQP iteration cap |
| `solver_eps_abs` | double | `0.001` | OSQP absolute convergence tolerance |
| `solver_eps_rel` | double | `0.001` | OSQP relative convergence tolerance |
| `warm_start` | bool | `true` | Reuse the previous (time-shifted) solution as the initial guess |

## Build & Launch

```bash
colcon build --packages-select vehicle_models mpc_controller

# Launch the full action pipeline with MPC instead of pure pursuit:
ros2 launch action_bringup action_mpc.launch.yaml
```

The package ships no launch file of its own — `action_mpc.launch.yaml` brings up the lattice planner, trajectory planner, the MPC node, and a lifecycle manager that autoconfigures/activates them. Package defaults from `mpc_controller/config/mpc_controller.yaml` are loaded first, then real-vehicle overrides from `action_bringup/config/action.yaml` (later wins).

## After Launching

1. **Verify lifecycle activation** — look for this log line from the node:

   ```
   Activated
   ```

   (preceded by `Configured: MPC control at 20.0 Hz`). The lifecycle manager autostarts the node; if it stays unconfigured, check the manager logs.

2. **Verify output is publishing:**

   ```bash
   ros2 topic hz /action/ackermann       # expect ~20 Hz (matches control_rate_hz)
   ros2 topic echo /action/is_idle        # false while tracking, true in idle/standby
   ```

3. **Check the idle gate** — if `/action/is_idle` is stuck `true`, one of these is the cause:
   - no trajectory on `/action/trajectory_planning/trajectory`, or it is empty
   - trajectory is stale (older than `idle_timeout_sec`)
   - behaviour string is empty, or equals `standby_msg` (with `disable_standby: false`)

4. **Confirm the pose lookup works** — a throttled warning means the TF tree is missing:

   ```
   Cannot get vehicle pose from TF (map -> base_footprint): ...
   ```

   Ensure a `map → base_frame` transform is being published.

5. **Visualize the prediction** — add the `/action/mpc/predicted_path` `nav_msgs/Path` in RViz; it should hug the planned trajectory ahead of the vehicle.

## Definition of Good Result

| Check | Expected |
|-------|----------|
| `/action/ackermann` rate | ~20 Hz, matching `control_rate_hz` |
| Predicted path vs. trajectory | Overlaps the planned trajectory; no oscillation or divergence at the tail |
| Steering output | Smooth, within `±max_steering_angle`, no chatter (governed by `w_dsteering` / `max_steering_rate`) |
| Speed command | Slows before curves and stop points; respects `max_speed` and per-point planner limits |
| `MPC solver failed` warnings | Absent in steady state (rare transient is tolerable; persistent means infeasible problem or too-tight tolerances) |
| No `Cannot get vehicle pose from TF` warnings after startup | `map → base_frame` transform is live |

## Internal Architecture

The package splits into a ROS-free solver core and a thin lifecycle node:

- **`MpcCore`** (`src/mpc_core.cpp`) — all the MPC math, no ROS dependencies (unit-tested in `test/test_mpc_core.cpp`).
  - `sample_reference()` — nearest-point search, fixed arc-length resampling, per-point `dt`, heading unwrapping, and curvature → steering / speed-profile → acceleration feedforward (the linearization operating point).
  - `solve()` — optional latency roll-forward of the initial state, then linearizes the bicycle model at each reference point.
  - `solve_qp()` — assembles the sparse Hessian `P`, gradient `q`, constraint matrix `A`, and bounds, then solves with OSQP. The decision vector is `z = [x_0…x_N, u_0…u_{N-1}]`. Constraint/Hessian sparsity patterns are kept fixed across solves (zeros are emitted explicitly) so OSQP can do in-place value updates; the solver is fully re-created only when the horizon length `N` changes. Warm-start shifts the previous primal solution forward one step.
    - **Speed cap is relaxed to a feasible braking envelope.** The speed upper bound is applied for `k=1..N` (not `k=0`: `v_0` is already pinned by the initial-state constraint) and is floored by the lowest speed reachable under the jerk/decel limits from the current speed and command. This keeps the QP feasible when the vehicle enters above the cap — instead of going infeasible and dropping into the open-loop fallback, it brakes at the comfort limit and converges under the cap over a few steps. Expect the predicted speed to briefly exceed `max_speed` right after entering hot; that is the envelope, not a bug.

- **`MpcControllerNode`** (`src/mpc_controller_node.cpp`) — lifecycle node and ROS plumbing.
  - **Lifecycle:** `on_configure` declares/loads params, sets up TF, subs/pubs, and constructs `MpcCore`; `on_activate` starts the control timer; `on_deactivate`/`on_cleanup`/`on_shutdown` release resources. Follows the same conventions as `PurePursuitNode` so it is a drop-in controller swap.
  - **Control loop:** a wall timer at `control_rate_hz` runs `control_callback`. It reads cached subscriber state (latest trajectory, behaviour, speed) plus a fresh TF pose lookup — it never blocks on messages. Idle/standby is gated first; otherwise it samples the reference, solves, publishes the Ackermann command, caches `prev_steering_`/`prev_accel_` (needed for the next cycle's rate/jerk constraints), and publishes the predicted path.
  - **Failure handling:** if the solver fails or the reference is shorter than 2 points, it falls back to holding the previous steering and coasting one control period under the previous command (`current_speed + prev_accel / control_rate_hz`), throttled-warning logged.

- **`vehicle_models::BicycleModel`** (`src/common/vehicle_models`) — shared kinematic bicycle model. `dynamics()` (continuous `f(x,u)`), `step()` (RK4 discretization), and `linearize()` (finite-difference Jacobians `A`, `B` plus affine term `g` so the model is exact at the operating point). The narrow interface lets a higher-fidelity model be swapped in without changing the controller.

## Tuning

The full tuning guide with recommended ranges is inline in `config/mpc_controller.yaml`. Quick reference:

- **Tracks loosely / cuts corners** — raise `w_lateral` (and `w_heading`).
- **Steering too jerky** — raise `w_dsteering` (primary knob) or tighten `max_steering_rate`.
- **Acceleration/braking too abrupt** — raise `w_daccel` (primary knob) or lower `max_jerk` / `max_accel` / `|max_decel|`.
- **Too sluggish / coasts** — raise `w_progress`, or switch to quadratic speed tracking via `w_speed`.
- **Brakes too late for curves/stops** — raise `horizon_distance` (sees further, slower solve).
- **Solver hits the iteration cap** (`MPC solver failed`) — raise `max_solver_iterations`, loosen `solver_eps_*`, or reduce `max_horizon_steps`.
- **High CPU** — lower `control_rate_hz`, `max_horizon_steps`, or `horizon_distance`; raise `point_spacing`.
- **Commands lag the vehicle** — set `latency_sec` to the measured actuation+compute delay.
