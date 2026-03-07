# WATO Trajectory Planner

The `trajectory_planner` package refines a geometric path into a velocity-profiled trajectory. It checks for obstacles along the path using a vehicle footprint and smoothly decelerates when a collision is imminent.

## Overview

The node subscribes to a path (from `local_planning`) and a costmap (from `world_modeling`). It interpolates the path at a fixed resolution, sweeps the vehicle footprint at each point, and finds the distance to the first lethal obstacle. Velocity at each point is then scaled linearly between `max_speed` (at `safe_distance`) and 0 (at `stop_distance`).

The lane speed limit from `/world_modeling/lanelet/lane_context` further caps velocity when available.

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
| `safe_distance` | 10.0 m | Distance to obstacle where deceleration begins. |
| `stop_distance` | 2.0 m | Distance to obstacle where vehicle must be fully stopped. |
| `max_speed` | 20.0 m/s | Maximum speed when no lanelet limit is available. |
| `interpolation_resolution` | 0.1 m | Point spacing along path for collision checking. |
| `footprint_frame` | `base_link` | Frame in which the footprint is defined. |
| `footprint_x_min` | -0.5 m | Rear extent of vehicle. |
| `footprint_x_max` | 3.5 m | Front extent of vehicle (front bumper). |
| `footprint_y_min` | -1.2 m | Right extent of vehicle. |
| `footprint_y_max` | 1.2 m | Left extent of vehicle. |

### Tuning Guide

1. **Car stops too early/late**: Adjust `stop_distance` or the `footprint_x_max` to match actual front bumper position.
2. **Car is too jerky**: Increase `safe_distance` to lengthen the deceleration ramp.
3. **Car clips obstacles on the sides**: Increase `footprint_y_min`/`footprint_y_max` to widen the safety corridor.
4. **High CPU usage**: Increase `interpolation_resolution` (e.g. 0.2 m), but avoid missing narrow obstacles.

## Troubleshooting

- **No trajectory output**: Verify `input_path` and `costmap` topics are publishing and remapped correctly.
- **"TrajectoryCore: Empty path"**: The upstream local planner is not producing a path.
- **Collisions not detected**: Confirm the costmap contains lethal cells (cost > 100) and that TF transforms between the path frame and costmap frame are available.
