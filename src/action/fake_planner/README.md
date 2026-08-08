# fake_planner

Replays a predefined maneuver as a trajectory so a controller can be exercised without the real planner stack. The same node runs in the CARLA sim and on the vehicle — only the topics, clock, and respawn handling differ, and those are set by the launch wrapper you pick.

## Overview

The node loads a maneuver (a JSON segment list under `maneuvers/`), expands it into a smooth, uniformly-spaced path, and publishes a rolling window of it as a `wato_trajectory_msgs/Trajectory` at a fixed rate — the same shape a real planner emits: a ~35 m horizon around the vehicle, re-sliced as it drives, not the whole route at once. Alongside the trajectory it publishes an `execute_behaviour` heartbeat so the controller leaves standby without the `world_modeling` behaviour stack, and a `trajectory_markers` MarkerArray mirroring `trajectory_planner`'s visualization.

An open maneuver ends in a braked stop; a `"closed": true` maneuver laps forever. The maneuver is laid out from the vehicle's pose at launch (**anchoring**), so the same file works wherever the car starts — unless it carries an absolute `start` pose (e.g. the oval), in which case anchoring is turned off and it publishes in fixed coordinates.

## Usage

Launch one of the two environment wrappers — not the rig directly — so the filename decides where the commands go.

```bash
# CARLA sim
ros2 launch fake_planner fake_planner_sim.launch.yaml

# Real vehicle (read the launch header before driving)
ros2 launch fake_planner fake_planner_vehicle.launch.yaml
```

Neither one drives on bringup: `start_on_activate` is false in both, so the stack comes up configured and activated but silent, with the controller held in standby, and a person releases it with the `start_trajectory` service below. The markers publish while held, so the path can be checked in Foxglove first. Pass `start_on_activate:=true` in sim to go back to driving straight off bringup. In sim a respawned ego re-arms the hold — after "respawn ego" the maneuver re-anchors at the new pose and waits to be released again.

Anchoring happens on the *first odom after launch*, not on release, so a vehicle that moves while held leaves the maneuver behind at the old pose and releasing it steers back there. Call `reset` before `start_trajectory` to re-anchor at where the vehicle actually is and republish the markers for a second look. It re-arms the hold rather than clearing it, so it is safe to make a habit of.

Pick the maneuver and controller with args:

```bash
ros2 launch fake_planner fake_planner_sim.launch.yaml maneuver:=brake_in_curve controller:=mpc
```

The controller's tuning follows the environment, so a hardware run measures the controller the car actually deploys: MPC reads `action_bringup`'s `action.yaml` everywhere, and pure pursuit reads its own package params in sim but that same `action.yaml` on the vehicle. Point it somewhere else with `pure_pursuit_config:=`.

Oval-track laps need the CARLA oval world. Nothing else changes on the command line — the maneuver is in absolute track coordinates and turns anchoring off for itself:

```bash
ros2 launch fake_planner fake_planner_sim.launch.yaml maneuver:=oval_track
```

Control a run through the node's services (fully-qualified name `/action/fake_planner/fake_planner_node`):

```bash
ros2 service call /action/fake_planner/fake_planner_node/start_trajectory std_srvs/srv/Trigger  # release
ros2 service call /action/fake_planner/fake_planner_node/stop_trajectory  std_srvs/srv/Trigger  # back to standby
ros2 service call /action/fake_planner/fake_planner_node/reset            std_srvs/srv/Trigger  # re-run from the current pose
```

## Maneuvers

A maneuver is a JSON file: a few top-level settings and a `segments` list. Each segment is authored in the local frame where the previous one ended (`u` forward along the current heading, `v` positive to the left), so segments simply chain. The whole path is sampled finely, resampled to `sample_spacing_m`, and — for open maneuvers — capped with a braked stop.

**Top-level fields:**

| Field | Default | Description |
|-------|---------|-------------|
| `name`, `description` | — | Informational only |
| `sample_spacing_m` | `1.0` | Uniform waypoint spacing (m). Match the map centreline (see note in `fake_planner_core.cpp`) |
| `default_speed` | `2.0` | Cruise speed (m/s), carried until a segment overrides it. All shipped maneuvers use `3.0` |
| `ramp_up_accel` | `1.0` | Acceleration (m/s²) of the launch ramp: an open maneuver starts at zero speed and builds to `default_speed` over `v²/2a` (4.5 m at the defaults), so the car pulls away gently instead of being commanded to full speed from the first waypoint. Keep the opening `straight` at least that long so the ramp finishes before any curvature. `0` disables it; ignored on closed maneuvers |
| `closed` | `false` | Lap forever (the window wraps); no stop pad or launch ramp |
| `start` | — | Absolute `{x, y, yaw}` start pose. Marks the maneuver as fixed geometry: under `anchoring:=auto` it publishes verbatim instead of being laid out from the vehicle |
| `segments` | *(required)* | Ordered list of `{type: {params}}`, one type per entry |

**Segment types** — each also takes optional `speed` and `end_speed`, which ramp linearly across the segment; speed carries into the next segment:

| Type | Params | Description |
|------|--------|-------------|
| `straight` / `dwell` | `length` | Advance along the heading. `dwell` is the same primitive, named to signal a hold |
| `shift` | `lateral`, `length` | Smooth raised-cosine lateral move, tangent to the heading at both ends |
| `slalom` | `amplitude`, `wavelength`, `cycles` | Sinusoidal weave, tapered onto/off the centreline |
| `arc` | `radius`, `angle` (deg), `dir` (`left`/`right`) | Constant-radius turn; also rotates the heading |

**Shipped maneuvers:**

| File | Description |
|------|-------------|
| `steady_state_slalom` | *(default)* Sustained constant-speed weave — lateral frequency response |
| `sine_with_dwell` | Swerve out, hold at the far offset, return — lateral transient and recovery |
| `brake_in_curve` | Brake through a constant-radius arc — steer-and-decelerate stability |
| `oval_track` | Closed 420.9 m lap in absolute oval coords; needs the CARLA oval world. Generated by `tools/generate_oval_track.py` — do not hand-edit |

See [DEVELOPING.md](DEVELOPING.md) to add a maneuver or a new segment type.

## ROS Interface

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `nav_msgs/Odometry` | Vehicle pose and speed: anchors the path, slides the window, and detects respawns |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `trajectory` | `wato_trajectory_msgs/Trajectory` | Rolling window of the maneuver |
| `execute_behaviour` | `behaviour_msgs/ExecuteBehaviour` | Heartbeat that keeps the controller out of standby (omitted when `publish_behaviour` is false) |
| `trajectory_markers` | `visualization_msgs/MarkerArray` | Speed-sized purple spheres and labels; published only while subscribed |

### Services

All `std_srvs/Trigger`, under the node's private namespace:

| Service | Action |
|---------|--------|
| `~/start_trajectory` | Begin or resume publishing |
| `~/stop_trajectory` | Stop publishing; the controller falls back to standby |
| `~/reset` | Rewind and re-run from the current pose (re-anchors when anchoring) |

## Configuration

Loaded from `config/params.yaml`. Topics, clock, and respawn are usually set by the launch wrapper; the rest live here.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `trajectory_topic` | string | `trajectory` | Trajectory output (before launch remaps) |
| `behaviour_topic` | string | `execute_behaviour` | Heartbeat output |
| `odom_topic` | string | `odom` | Odometry source |
| `frame_id` | string | `odom` | Frame the trajectory is published in; must be TF-connected to the controller's `rear_axle` (`map`/`odom` both work) |
| `publish_rate_hz` | double | `10.0` | Republish rate; keep well under the controller's `idle_timeout_sec` (2.0 s) |
| `publish_behaviour` | bool | `true` | Publish the heartbeat. False to use a real behaviour source |
| `behaviour` | string | `lane_follow` | Heartbeat behaviour string (must not be the controller's standby string) |
| `anchoring` | string | `auto` | `auto` \| `relative` \| `absolute`. `auto` reads it off the maneuver: a file with an absolute `start` publishes verbatim, everything else is laid out from the launch pose |
| `start_on_activate` | bool | `true` | Publish on activate. False holds until `start_trajectory` — which is what every launch passes, so nothing drives on bringup unless you set `start_on_activate:=true` |
| `maneuver_file` | string | *(required)* | Path to the maneuver JSON; set by the launch from `maneuver:=` |
| `marker_pub_topic` | string | `trajectory_markers` | Marker output |
| `horizon_m` | double | `35.0` | Window length ahead of the vehicle (m) |
| `trail_m` | double | `2.0` | Window length behind the vehicle (m) |
| `respawn_jump_m` | double | `5.0` | Odom jump treated as a respawn, after which the maneuver re-anchors. 0 disables |

## License

Copyright (c) 2025-present WATonomous. All rights reserved.
Licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).
