# Freeroam Planner

ROS2 lifecycle node that generates a trajectory for autonomous free-roaming navigation using a costmap and a goal point, tracked by a pure pursuit controller.

## Overview

The freeroam planner receives a costmap and a goal point, computes a drivable trajectory, and publishes it for the pure pursuit controller to track. It is managed by the WATonomous lifecycle manager alongside the pure pursuit node.

**Current Status**: Functional freeroam trajectory generation. Integrates with world modeling for costmap input and the ackermann pure pursuit controller for path tracking.

## ROS Interface

### Subscribed Topics

| Topic | Remapped From | Type | Description |
|-------|--------------|------|-------------|
| `costmap` | `/world_modeling/costmap` | `nav_msgs/OccupancyGrid` | Occupancy grid for obstacle-aware planning |
| `goal_point` | `/goal_point_stamped` | `geometry_msgs/PointStamped` | Target goal point for the planner |

### Published Topics

| Topic | Remapped To | Type | Description |
|-------|------------|------|-------------|
| `trajectory` | `/action/freeroam_planner/trajectory` | `nav_msgs/Path` | Planned trajectory for the pure pursuit controller |

## Algorithm Details

### A* Path Search
The planner runs A* on the occupancy grid to find the shortest collision-free path from the vehicle's current position to the goal. Grid cells with a cost value at or above `obstacle_threshold` are treated as impassable. With `allow_diagonal` enabled, the search expands to 8-connected neighbours; otherwise only 4-connected (up, down, left, right).

### Costmap Inflation
Before planning, the costmap is pre-processed to inflate all obstacle cells by `inflation_radius_m_`. This creates a safety buffer around obstacles so that the computed path keeps the vehicle body clear of detected hazards.

### Replanning
The planner runs at a fixed rate (`planning_rate_hz`) and replans from the current vehicle pose on every cycle, allowing it to react to new obstacles appearing in the costmap.

## Configuration

Parameters are loaded from `config/params.yaml` under the namespace `action/freeroam_planner/freeroam_planner_node/ros__parameters`.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `base_frame` | string | `"base_footprint"` | Robot base frame for planning |
| `max_speed` | double | `5.0` | Maximum speed along the trajectory (m/s) |
| `goal_tolerance` | double | `1.0` | Distance threshold to consider the goal reached (m) |
| `obstacle_threshold` | int | `50` | Minimum occupancy grid cost value considered an obstacle |
| `inflation_radius_m_` | double | `1.6` | Radius around obstacles to inflate as impassable (m) |
| `allow_diagonal` | bool | `true` | Allow diagonal moves during path search |
| `planning_rate_hz` | double | `2.0` | Frequency at which the planner replans (Hz) |

## Dependencies

- ROS 2 (tested on Humble)
- `nav_msgs`, `geometry_msgs` (standard message types)
- `wato_trajectory_msgs` (custom trajectory message)

## License

Copyright (c) 2025-present WATonomous. All rights reserved.
Licensed under the [Apache License, Version 2.0](http://www.apache.org/licenses/LICENSE-2.0).
