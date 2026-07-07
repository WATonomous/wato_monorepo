# WATO Trajectory Planner

The `trajectory_planner` package refines a geometric path into a velocity-profiled trajectory. Before profiling, it can laterally deform the path around obstacles with a discrete elastic band, and it checks for obstacles along the (possibly deformed) path using a vehicle footprint, smoothly decelerating when a collision is imminent.

## Overview

The node subscribes to a path (from `local_planning`) and a costmap (from `world_modeling`). It transforms the path into the costmap frame, optionally runs the elastic band deformation stage (see below), interpolates the path at a fixed resolution, sweeps the vehicle footprint at each point, and finds the distance to the first lethal obstacle. Velocity at each point is then limited by kinematics-based braking: `v = sqrt(2 * max_tangential_accel * braking_distance)`, ensuring the vehicle can always stop in time given its current speed and max deceleration.

The lane speed limit from `/world_modeling/lanelet/lane_context` further caps velocity when available.

## Elastic band obstacle avoidance

Before velocity profiling, the node can run a discrete elastic band (Quinlan-Khatib style) over the path to nudge it laterally around obstacles that intersect it in the costmap, instead of only slowing down and stopping. The band is a set of interior path points, each pulled by three forces every iteration:

- **Smoothing force**: contracts the band toward a smooth curve through its neighbours (`eb_smooth_weight`).
- **Obstacle repulsion force**: pushes points away from the nearest lethal costmap cell within `eb_influence_radius`, falling off linearly with distance (`eb_obstacle_weight`).
- **Anchor force**: pulls each point back toward its original (un-deformed) position (`eb_anchor_weight`).

Each point is stepped by `eb_step_size` times the combined force, then clamped so it never strays more than `eb_max_deviation` from the original path. The first and last points never move. Iteration stops early once the largest per-point displacement in an iteration drops below `eb_convergence_tol`, or after `eb_max_iterations`. Pose orientations are recomputed from the deformed tangent direction afterward.

The deformed path feeds directly into the existing collision check and velocity profiler, so if the band cannot find a clear deformation (e.g. a fully blocked corridor), `find_first_collision` still detects the obstacle on the deformed path and the vehicle stops before it — the elastic band is a best-effort refinement layered on top of the existing stop-before-collision safety behaviour, never a replacement for it.

The deformed path is visualized as a green line strip (`elastic_band_path` marker namespace) alongside the speed-colored spheres, so the deviation from the original path is visible in RViz.

## Usage

```bash
ros2 launch trajectory_planner trajectory_planner.launch.yaml
```

Topic remappings:
- `input_path` → `/action/local_planning/path`
- `costmap` → `/world_modeling/costmap`
- `trajectory` → `/action/trajectory_planning/trajectory`
- `lane_context` → `/world_modeling/lanelet/lane_context`

## Visualization

Publishes `visualization_msgs/MarkerArray` on `~trajectory_markers`. Each point is rendered as a purple sphere whose diameter scales with target speed (larger = faster). The deformed path geometry is additionally rendered as a green line strip (`elastic_band_path` namespace).

## Configuration

Parameters are defined in `config/trajectory_planner_params.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stop_distance` | 2.0 m | Distance to obstacle where vehicle must be fully stopped. |
| `max_speed` | 20.0 m/s | Maximum speed when no lanelet limit is available. |
| `max_tangential_accel` | 1.0 m/s^2 | Comfort braking deceleration for normal obstacle avoidance. |
| `max_emergency_accel` | 5.0 m/s^2 | Emergency braking deceleration when comfort braking is insufficient. |
| `max_lateral_accel` | 0.5 m/s^2 | Maximum lateral acceleration to slow down in curves. |
| `interpolation_resolution` | 0.1 m | Point spacing along path for collision checking. |
| `footprint_frame` | `base_link` | Frame in which the footprint is defined. |
| `footprint_x_min` | -0.5 m | Rear extent of vehicle. |
| `footprint_x_max` | 3.5 m | Front extent of vehicle (front bumper). |
| `footprint_y_min` | -1.2 m | Right extent of vehicle. |
| `footprint_y_max` | 1.2 m | Left extent of vehicle. |
| `elastic_band_enabled` | true | Enables the elastic band path deformation stage described above. |
| `eb_max_iterations` | 50 | Max gradient-descent iterations per deformation. |
| `eb_step_size` | 0.2 | Gradient step applied to the combined force each iteration. |
| `eb_smooth_weight` | 0.5 | Internal contraction force pulling points onto a smooth curve. |
| `eb_obstacle_weight` | 1.5 | Repulsion force pushing points away from nearby obstacles. |
| `eb_anchor_weight` | 0.1 | Force pulling points back toward the original path. |
| `eb_influence_radius` | 2.5 m | Obstacle repulsion falls off to zero beyond this range. |
| `eb_max_deviation` | 1.5 m | Cap on lateral deviation from the original path. |
| `eb_convergence_tol` | 0.01 m | Stop iterating once max per-iteration displacement is below this. |

### Tuning Guide

1. **Car stops too early/late**: Adjust `stop_distance` or the `footprint_x_max` to match actual front bumper position.
2. **Car is too jerky**: Decrease `max_tangential_accel` for gentler braking.
3. **Car clips obstacles on the sides**: Increase `footprint_y_min`/`footprint_y_max` to widen the safety corridor.
4. **High CPU usage**: Increase `interpolation_resolution` (e.g. 0.2 m), but avoid missing narrow obstacles.
5. **Band should dodge obstacles earlier/wider**: Raise `eb_obstacle_weight` and/or `eb_influence_radius` so points feel the repulsion sooner and more strongly.
6. **Band leaves the intended lane/corridor**: Lower `eb_max_deviation` to bound how far it is allowed to stray from the original path — when a clear deformation would require exceeding it, the band stops short and the existing stop-before-collision fallback (`stop_distance`, `max_emergency_accel`) takes over instead.
7. **Disable path deformation entirely**: Set `elastic_band_enabled: false` to restore the original velocity-profiler-only behaviour.

## Troubleshooting

- **No trajectory output**: Verify `input_path` and `costmap` topics are publishing and remapped correctly.
- **"TrajectoryCore: Empty path"**: The upstream local planner is not producing a path.
- **Collisions not detected**: Confirm the costmap contains lethal cells (cost > 100) and that TF transforms between the path frame and costmap frame are available.
