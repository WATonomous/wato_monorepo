# Fake planner

This lifecycle node replays reviewed maneuver JSON without the real planner stack.
It imports Brian Zheng's fake planner from PR 487, commit
`f19fbad28cba3eb9bb5c6053046831c3a33f84fa`, with simulation session controls added.
The action image discovers the package through its existing `COPY src/action` and
colcon build; it needs no separate image or Dockerfile.

## Simulation usage

The runner's `controller_mpc` preset uses the following command. The launch owns
one fake planner, one MPC controller and one lifecycle manager. Do not launch the
full action stack beside it, since that would add a second trajectory source and
controller.

```bash
ros2 launch fake_planner fake_planner_sim.launch.yaml \
  controller:=mpc frame_id:=map start_on_activate:=false
```

The planner configures and activates without sending trajectory or behavior
commands. Once odometry anchors the reference, `/sim/reference_path`
(`nav_msgs/msg/Path`, reliable and transient local) contains the entire maneuver.
The rolling preview markers remain on
`/action/fake_planner/trajectory_markers`.

```bash
ros2 service call /action/fake_planner/start_trajectory std_srvs/srv/Trigger
ros2 service call /action/fake_planner/stop_trajectory std_srvs/srv/Trigger
ros2 service call /action/fake_planner/reset std_srvs/srv/Trigger
```

`/action/fake_planner/status` is a reliable transient-local String with `ready`,
`running`, `finished` or `stopped`. Ready requires an odometry-anchored reference.
Finished requires actual movement, arrival near the end of the open maneuver and
speed below the configured threshold. A successful start service is insufficient
to establish completion. Reset clears the anchor and movement/completion latches;
the next odometry sample anchors a fresh reference. The runner also cycles the
controller lifecycle so its prior trajectory and control history are discarded.

Publication and release ramp timing follow the ROS clock. Freezing CARLA `/clock`
therefore freezes trajectory and behavior publication instead of refreshing stale
commands on a wall timer. The CARLA world authority also brakes/holds the ego
while the session is paused or preparing.

The simulation wrapper selects `action_sim.yaml` for MPC, `/ego/odom`,
`/action/trajectory_planning/trajectory` and
`/carla/ackermann_control/command`. `mpc_config`, `maneuver`, `maneuver_file` and
`anchoring` are launch arguments. The shared wrapper supports the original pure
pursuit controller for manual use; the runner selects MPC explicitly.

## Maneuver format

`maneuvers/steady_state_slalom.json` is the shipped controller experiment:
6 metres straight, five sinusoidal cycles with 1 metre amplitude and 12 metre
wavelength, then 6 metres of braking straight at an authored cruise speed of
3 m/s. The world definition requires at least 100 metres of verified straight
runout. The original `brake_in_curve` and `sine_with_dwell` assets remain available
for manual controller work.

JSON defines `sample_spacing_m`, `default_speed` and an ordered `segments` list.
The core expands straight, arc, slalom, shift and dwell segments, applies launch
and braking ramps, and publishes a rolling horizon around the vehicle. Relative
maneuvers anchor at the first odometry pose; an absolute `start` preserves authored
coordinates. `anchoring:=relative` or `anchoring:=absolute` overrides that choice.
Core parsing and geometry tests from the original contribution are retained in
`test/test_fake_planner_core.cpp`; live and build validation are operator work.
