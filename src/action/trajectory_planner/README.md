# WATO Trajectory Planner

The `trajectory_planner` package refines a geometric path into a velocity-profiled trajectory. It laterally nudges the path around slight costmap obstacles, checks for obstacles along the (possibly nudged) path using a vehicle footprint, and smoothly decelerates when a collision is imminent.

## Overview

The node subscribes to a path (from `local_planning`) and a costmap (from `world_modeling`). It first runs an elastic lateral deformation pass (see below) to nudge the path around slight obstacles, then interpolates the deformed path at a fixed resolution, sweeps the vehicle footprint at each point, and finds the distance to the first lethal obstacle. Velocity at each point is then limited by kinematics-based braking: `v = sqrt(2 * max_tangential_accel * braking_distance)`, ensuring the vehicle can always stop in time given its current speed and max deceleration.

The lane speed limit from `/world_modeling/lanelet/lane_context` further caps velocity when available.

## Elastic Lateral Deformation

Before velocity profiling, points whose costmap cost exceeds `deformation_trigger_cost` search for the cheapest nearby offset within `max_lateral_shift`, then relax toward that target and toward their neighbors' average over several iterations — smoothing out the shift and tapering it back to the centerline away from the obstacle. The first and last path points never move, and any step that would land on a lethal cell is rejected. A fully clear path is a no-op; an obstacle too wide to route around falls back to the stop-before-collision braking below.

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

Publishes `visualization_msgs/MarkerArray` on `~trajectory_markers`. Each point is rendered as a purple sphere whose diameter scales with target speed (larger = faster).

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
| `deformation_trigger_cost` | 10.0 | Costmap cost above which a point is considered for deformation. |
| `max_lateral_shift` | 0.5 m | How far from centre a triggered point may search/move. |
| `lateral_search_step` | 0.1 m | Candidate offset resolution for the target search. |
| `deformation_iterations` | 20 | Max smoothing/relaxation iterations. |
| `deformation_convergence_tol` | 0.01 m | Stop early once max per-iteration displacement is below this. |
| `smoothing_gain` | 0.3 | Pull toward the average of neighbouring offsets each iteration. |
| `pull_gain` | 0.5 | Pull toward the target offset found by the search each iteration. |
| `max_step` | 0.05 m | Cap on how far an offset may change in a single iteration. |

### Tuning Guide

1. **Car stops too early/late**: Adjust `stop_distance` or the `footprint_x_max` to match actual front bumper position.
2. **Car is too jerky**: Decrease `max_tangential_accel` for gentler braking.
3. **Car clips obstacles on the sides**: Increase `footprint_y_min`/`footprint_y_max` to widen the safety corridor.
4. **High CPU usage**: Increase `interpolation_resolution` (e.g. 0.2 m), but avoid missing narrow obstacles.
5. **Car doesn't nudge around minor obstacles**: Lower `deformation_trigger_cost` so it reacts to lighter costmap inflation, or raise `max_lateral_shift` if it's finding an obstacle but has nowhere clear to go.
6. **Nudge is too abrupt**: Lower `max_step` or `pull_gain`, or raise `smoothing_gain`.
7. **Nudge doesn't fully settle**: Raise `deformation_iterations`, or lower `deformation_convergence_tol` so it isn't declared "good enough" too early.

## Troubleshooting

- **No trajectory output**: Verify `input_path` and `costmap` topics are publishing and remapped correctly.
- **"TrajectoryCore: Empty path"**: The upstream local planner is not producing a path.
- **Collisions not detected**: Confirm the costmap contains lethal cells (cost > 100) and that TF transforms between the path frame and costmap frame are available.
