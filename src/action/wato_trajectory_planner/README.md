# WATO Trajectory Planner

The `wato_trajectory_planner` package implements a longitudinal trajectory planner that refines a geometric path into a velocity-profiled trajectory. It ensures collision avoidance by scaling velocity based on the distance to obstacles detected in the costmap.

## Overview

The planner subscribes to a geometric path (from `local_planning`) and a costmap (from `world_modeling`). It checks for collisions along the path using a footprint-based check and generates a `wato_trajectory_msgs/Trajectory` where each point has a target velocity.

**Key Features:**
- **Linear Velocity Scaling**: Smoothly decelerates from `max_speed` to 0 as obstacles approach.
- **Dynamic Speed Limit**: Respects the speed limit of the current lanelet (via `/world_modeling/lanelet/lane_context`).
- **Footprint Checking**: Checks a radius around interpolated path points to ensure the vehicle body clears obstacles.
- **Configurable**: Safety distances, vehicle geometry, and interpolation resolution are all tunable.

## Usage

### Launching

To launch the planner in isolation with default parameters:
```bash
ros2 launch wato_trajectory_planner trajectory_planner.launch.py
```

This launch file remaps:
- `input_path` -> `/action/local_planning/path`
- `costmap` -> `/world_modeling/costmap`
- `trajectory` -> `/action/trajectory_planning/trajectory`
- `lane_context` -> `/world_modeling/lanelet/lane_context`

### Integration

To integrate with the full stack, ensure this node is included in `action_launch.yaml` or similar bringup configurations.

## Visualization

The planner publishes a `visualization_msgs/MarkerArray` on the topic `~trajectory_markers` (remapped to `/action/trajectory_planning/trajectory_markers`).

- **Markers**: Spheres along the trajectory path.
- **Size**: The diameter of each sphere represents the target speed at that point.
  - **Small**: Low speed (approaching stop).
  - **Large**: High speed (up to max limit).
- **Color**: Uniform purple for clarity.

## Configuration

Parameters are defined in `config/trajectory_planner_params.yaml`.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `safe_distance` | 10.0 m | Distance to obstacle where deceleration begins. |
| `stop_distance` | 2.0 m | Distance to obstacle where vehicle must be fully stopped (relative to front bumper). |
| `max_speed` | 5.0 m/s | Maximum target speed if not constrained by lanelet/curvature. |
| `vehicle_front_offset` | 2.5 m | Distance from `base_link` (path origin) to the front bumper. |
| `footprint_radius` | 1.2 m | Radius around path points to check for collisions (approx vehicle width/2 + buffer). |
| `interpolation_resolution` | 0.1 m | Spacing of points for collision checking. |

### Tuning Guide

1.  **Car stops too early/late**: Adjust `vehicle_front_offset` to match the actual vehicle geometry.
2.  **Car is too jerky**: Increase `safe_distance` to provide a longer deceleration ramp.
3.  **Car hits obstacles on sides**: Increase `footprint_radius` to widen the safety corridor.
4.  **High CPU usage**: Increase `interpolation_resolution` (e.g., to 0.2m), but be careful not to miss small obstacles.

## Troubleshooting

-   **No Trajectory Output**: Check if `input_path` and `costmap` topics are publishing and correctly remapped.
-   **"TrajectoryCore: Empty path"**: The upstream local planner is not producing a path.
-   **Collisions not detected**: Verify `costmap` has data (lethal obstacles > 100) and that `tf2` transforms between path frame and costmap frame are valid.
