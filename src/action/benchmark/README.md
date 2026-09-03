# Controller Benchmark

Closed-loop comparison of `mpc_controller` against `ackermann_pure_pursuit` on identical
scenarios and an identical plant model.

The benchmark **links the production MPC sources directly** — `mpc_core.cpp` and
`bicycle_model.cpp` — so the MPC numbers come from the real controller rather than a
reimplementation. The pure pursuit law is transcribed from `pure_pursuit_node.cpp`
(`controlCallback`), because that logic is welded to its ROS node and cannot be linked
standalone.

Gains for both controllers are the as-deployed values from
`src/action/action_bringup/config/action.yaml`.

## Running

From inside the action container, with `src/action` mounted at `/ws/src/action`:

```bash
# one-time: build the two dependencies the benchmark needs
colcon build --packages-select osqp_eigen_vendor wato_trajectory_msgs \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

# build the benchmark and run every scenario
bash src/action/benchmark/build_and_run.sh
```

Run a single scenario, one `MpcCore` per process (matching the node):

```bash
/tmp/bench/bench lane_change out/lane_change.json
```

Scenarios: `straight`, `curve_r30`, `curve_nowrap`, `lane_change`, `double_lane_change`,
`stop_line`.

Each run emits JSON with cross-track RMS and max, heading error, steering rate and total
variation, peak lateral acceleration, speed statistics, solver failure counts, and
per-cycle timing percentiles.

## Plant model

Both controllers drive a nonlinear kinematic bicycle at 20 Hz. The Ackermann speed command
is tracked with a first-order lag (`SPEED_LAG_TAU = 0.3 s`), identical for both, so the
comparison is fair — but absolute speed figures depend on that constant.

There is no tyre slip, actuator dynamics, localization noise, or planner in the loop. This
isolates the controllers from everything else; it is why both are exact on the straight.
Treat the output as bench numbers, not vehicle numbers.

## Known findings

`curve_nowrap` exists to separate two defects that both degrade curve tracking: it is the
same R = 30 m curve as `curve_r30`, truncated so the reference heading never crosses ±π.
Comparing the two isolates the unwrapped heading cost in `mpc_core.cpp` from the
map-frame position cost.

Note that `mpc_core.cpp` currently fails inside `initSolver()` on horizon-length changes,
and does not recover. To measure the MPC's actual tracking capability, the QP data
(`q`, `lower`, `upper`, `P`, `A_mat`) needs a lifetime that outlives `solve_qp` — OsqpEigen
stores pointers to them rather than copying.
