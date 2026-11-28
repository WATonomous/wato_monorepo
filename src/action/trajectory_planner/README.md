# WATO Trajectory Planner

The `trajectory_planner` package refines a geometric path into a velocity-profiled trajectory. Before profiling, it can deterministically shift the path laterally around lethal costmap obstacles. It then checks the complete path with the vehicle footprint and smoothly decelerates when a collision remains.

## Overview

The node subscribes to a path (normally from `lattice_planning`) and a costmap (from `world_modeling`). It transforms the path into the costmap frame, optionally runs the elastic trajectory stage described below, sweeps the vehicle footprint along the result, and finds the distance to the first lethal obstacle. Velocity at each point is limited by kinematics-based braking: `v = sqrt(2 * max_tangential_accel * braking_distance)`.

The lane speed limit from `/world_modeling/lanelet/lane_context` further caps velocity when available.

## Naive elastic trajectory obstacle avoidance

When enabled, the planner:

1. Resamples the input at `interpolation_resolution`, preserving the exact endpoint positions.
2. Groups intersecting lethal cells into collision clusters. Clusters whose transition regions overlap are merged so a maneuver cannot oscillate between left and right.
3. Searches left and right in `eb_lateral_search_step` increments, up to `eb_max_deviation`. It takes the smallest clear shift and uses `eb_preferred_side` only for a tie.
4. Applies cubic smoothstep transitions over `eb_transition_distance`, holds a constant offset beside the cluster, and returns to the original centerline. Both endpoints remain anchored.
5. Recomputes yaw from the deformed geometry and validates the full swept footprint with `eb_clearance_margin`.

Deformation is atomic. If any cluster has no valid offset, lacks room for its transitions, or the final swept path collides, the entire deformation is discarded. Velocity profiling then uses the resampled centerline, so the existing stop-before-obstacle behavior remains the fallback. Disabling `elastic_band_enabled` bypasses both deformation and resampling, preserving the legacy geometry and waypoint count.

Only costmap cells with lethal cost `100` trigger lateral deformation. Costs `1–99` continue to reduce the velocity profile without causing a path shift.

This is deliberately a naive, costmap-only planner. It does not consult HD-map lane boundaries, assess whether a shift is legal, predict dynamic obstacles, or remember a side choice between planning cycles. Keep `eb_max_deviation` conservative and do not treat a geometrically clear shift as proof that it is road-legal.

The selected geometry is visualized as a green line strip (`elastic_band_path` marker namespace) alongside the speed-colored spheres.

## Path smoothing and curvature

Lanelet centrelines are polylines. Resampling them finely (e.g. 0.05 m) concentrates the heading change at each map vertex into one or two short segments. The per-segment curvature estimate then spikes, and the lateral-acceleration limit collapses the speed profile at every kink: a 2° kink caps speed at about 2.4 m/s with `max_lateral_accel: 2.0`.

When `path_smoothing_enabled` is true, the path goes through two steps before obstacle deformation:

1. **Elastic band smoothing.** The path is resampled to a coarse band at `smoothing_spacing`. Each interior band point may move only along its original normal, by at most `smoothing_max_deviation`. Projected gradient descent minimises the band's *bending energy* (sum of squared second differences) plus a small anchor term. Classic elastic-band *tension* (first differences) is not used, because it pulls every curve inward and cuts corners. Bending energy removes kinks but leaves constant-radius arcs in place. The two points at each end are held fixed. The relaxed band is densified with a Catmull-Rom spline and resampled at `interpolation_resolution`.
2. **Windowed curvature.** When `curvature_window > 0`, the lateral-acceleration speed limit uses the circle through the points about `curvature_window` metres behind and ahead of each pose, instead of the yaw difference between adjacent poses.

The pipeline is: resample → smooth → obstacle deformation → collision check → velocity profile. Smoothing is purely geometric: it ignores the costmap. Deviation is small and bounded, and the collision check and braking still run on the final geometry.

## Usage

```bash
ros2 launch trajectory_planner trajectory_planner.launch.yaml
```

Topic remappings:

- `input_path` → `/action/lattice_planning/path`
- `costmap` → `/world_modeling/costmap`
- `trajectory` → `/action/trajectory_planning/trajectory`
- `lane_context` → `/world_modeling/lanelet/lane_context`

## Visualization

The node publishes `visualization_msgs/MarkerArray` on `~trajectory_markers`. Each point is rendered as a purple sphere whose diameter scales with target speed. The selected path geometry is additionally rendered as a green line strip in the `elastic_band_path` namespace.

## Configuration

Parameters are defined in `config/trajectory_planner_params.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `stop_distance` | 2.0 m | Distance to obstacle where vehicle must be fully stopped. |
| `max_speed` | 20.0 m/s | Maximum speed when no lanelet limit is available. |
| `max_tangential_accel` | 1.0 m/s² | Comfort braking deceleration. |
| `max_emergency_accel` | 5.0 m/s² | Emergency braking deceleration when comfort braking is insufficient. |
| `max_lateral_accel` | 0.5 m/s² | Maximum lateral acceleration used to limit curve speed. |
| `interpolation_resolution` | 0.1 m | Spacing of the resampled path when elastic planning is enabled. |
| `footprint_frame` | `base_link` | Frame in which the footprint extents are defined. |
| `footprint_x_min` | -0.5 m | Rear extent of the vehicle. |
| `footprint_x_max` | 3.5 m | Front extent of the vehicle. |
| `footprint_y_min` | -1.2 m | Right extent of the vehicle. |
| `footprint_y_max` | 1.2 m | Left extent of the vehicle. |
| `elastic_band_enabled` | true | Enables resampling and deterministic lateral deformation. |
| `eb_max_deviation` | 0.8 m | Maximum lateral shift from the input centerline. |
| `eb_lateral_search_step` | 0.1 m | Increment used to search for the smallest clear left/right offset. |
| `eb_clearance_margin` | 0.3 m | Extra lateral footprint clearance used during search and final validation. |
| `eb_transition_distance` | 5.0 m | Cubic smoothstep distance before and after each collision cluster. |
| `eb_preferred_side` | `left` | Tie-break when equal left and right offsets are valid (`left` or `right`). |
| `path_smoothing_enabled` | true | Enables elastic band smoothing of the path geometry. |
| `smoothing_spacing` | 1.0 m | Spacing of the coarse band that is relaxed. |
| `smoothing_max_deviation` | 0.3 m | Maximum lateral displacement of any band point. |
| `smoothing_iterations` | 200 | Maximum relaxation iterations. |
| `smoothing_step` | 0.1 | Gradient step; clamped to 0.125 for stability. |
| `smoothing_anchor_weight` | 0.01 | Pull back toward the original path. |
| `smoothing_tolerance` | 1e-4 m | Stop once the largest per-iteration move is below this. |
| `curvature_window` | 1.0 m | Half-window for curvature-limited speed; `<= 0` uses the legacy per-segment estimate. |

### Tuning guide

1. **Car stops too early or late**: Adjust `stop_distance` or `footprint_x_max` to match the front bumper.
2. **Car is too jerky**: Decrease `max_tangential_accel` for gentler braking.
3. **Car clips obstacles on the sides**: Correct the footprint extents or increase `eb_clearance_margin`.
4. **High CPU usage**: Increase `interpolation_resolution`, while ensuring narrow obstacles are not skipped.
5. **The maneuver is too abrupt**: Increase `eb_transition_distance`. If the path lacks enough room, deformation will be rejected and braking will take over.
6. **The path shifts too far**: Lower `eb_max_deviation`; an obstacle that cannot be cleared within the bound will use the braking fallback.
7. **Disable path deformation entirely**: Set `elastic_band_enabled: false` to restore velocity profiling on the original geometry and waypoint count.

## Troubleshooting

- **No trajectory output**: Verify that the input path and costmap topics are publishing and remapped correctly.
- **Collisions are not detected**: Confirm the costmap contains lethal cells with cost `100` and that TF is available between the path and costmap frames.
- **Planner brakes instead of shifting**: Check for a full-width obstacle, insufficient transition room near an endpoint, a required shift greater than `eb_max_deviation`, or a collision along either smoothstep ramp.
