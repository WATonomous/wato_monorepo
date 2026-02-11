# Local Planning Module

Smooth, kinematically-feasible local path generation using cubic spiral optimization with map-aware corridor sampling and cost-based path selection.

## Overview

Generates drivable paths by:
1. Receiving ego vehicle pose and local lanelet corridor from world modeling
2. Sampling terminal points at multiple lookahead horizons across available lanes
3. Fitting cubic spirals between current pose and each terminal using damped Newton optimization
4. Scoring paths based on curvature smoothness, lateral movement, and lane preference
5. Publishing lowest-cost path for tracking controller

**Current Status**: Functional spiral path generation with 3-DOF pose optimization. Integrates with behavior planner for lane preference and lanelet map for corridor constraints.

## ROS Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `nav_msgs/Odometry` | Ego vehicle odometry (position, orientation, curvature) |
| `lanelet_ahead` | `lanelet_msgs/LaneletAhead` | Local lanelet corridor with centerlines and topology |
| `execute_behaviour` | `behaviour_msgs/ExecuteBehaviour` | Preferred lanelet IDs from behavior planner |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Selected path as sequence of poses |
| `planned_paths_markers` | `visualization_msgs/MarkerArray` | All candidate paths and terminal points (visualization) |
| `final_path_markers` | `visualization_msgs/Marker` | Selected path (visualization) |

## Architecture

**Lifecycle node with separate algorithm core**:

- **local_planner_node**: ROS 2 lifecycle node handling subscriptions, publications, and corridor processing
- **local_planner_core**: Pure algorithm library for spiral generation, cost evaluation, and path selection
  - `generate_path()`: Damped Newton optimizer fitting cubic spirals k(s) = c0 + c1s + c2s^2 + c3s^3
  - `path_cost_function()`: Evaluates curvature change penalties, lateral movement cost, and lane preference
  - `get_lowest_cost_path()`: Selects minimum-cost path from candidate set
  - `compute_jacobian_3dof()`: Finite-difference Jacobian for 3-DOF pose error (x, y, theta)

## Algorithm Details

### Cubic Spiral Generation
Paths are represented as cubic spirals where curvature varies smoothly:
- k(s) = c0 + c1s + c2s^2 + c3s^3
- Optimization variables: intermediate curvatures k1, k2 (at 1/3 and 2/3 arc length), total arc length sf
- Solved via damped Newton's method with curvature clamping to vehicle limits
- Forward Euler integration discretizes spirals into waypoints

### Cost Function
Paths scored on:
- **Curvature change**: Penalizes exceeding max curvature change threshold (vehicle dynamics limits)
- **Lateral movement**: Weighted by absolute curvature (encourages straight paths)
- **Lane preference**: Fixed penalty for non-preferred lanes (from behavior planner)

### Corridor Sampling
- Extracts centerlines from current and adjacent lanelets
- Samples terminal points at configurable lookahead distances (default: 10m, 15m, 20m)
- Handles lane splits by branching ego lane successors

## Configuration

Parameters in `config/local_planner_params.yaml`:

**Corridor Sampling**:
- `num_horizons`: Number of lookahead distances (default: 3)
- `lookahead_distances`: Arc length sampling points in meters (default: [10.0, 15.0, 20.0])

**Path Generation (Optimization)**:
- `max_iterations`: Newton solver max iterations (default: 20)
- `path_steps`: Spiral discretization steps (default: 20)
- `convergence_tolerance`: 3-DOF pose error threshold (default: 0.25 m/rad)
- `newton_damping`: Damping factor for Newton updates (default: 0.7)
- `max_step_size`: Maximum parameter update per iteration (default: 1.0)

**Cost Function**:
- `cm_lateral_movement_weight`: Curvature penalty weight (default: 2.0)
- `cm_physical_limits_weight`: Curvature change violation penalty (default: 4.0)
- `cm_preferred_lane_cost`: Fixed cost for non-preferred lanes (default: 20.0)
- `cm_max_curvature_change`: Max curvature change threshold rad/m (default: 0.1)

## Current Limitations

- **3-DOF optimization only**: Curvature at terminal point is not enforced (optimizes x, y, theta but not kappa)
- **No obstacle avoidance**: Paths follow lanelet centerlines without dynamic object consideration
- **Single-threaded**: All candidate paths generated sequentially
- **No path smoothing post-processing**: Relies solely on cubic spiral smoothness
- **Hard-coded horizon count**: Splitting behavior maximises at a 3-lane corridor (left/ego/right)
- **Doesn't Use Strict Frenet Frame**: Stores all points as (x, y, theta, kappa) instead of frenet frame 
- **No convergence diagnostics**: Returns empty path on failure without intermediate results

## Dependencies

- ROS 2 (tested on Humble)
- Eigen3 (matrix operations for optimization)
- `lanelet_msgs`, `behaviour_msgs` (custom message types)
- `tf2_geometry_msgs` (quaternion conversions)
