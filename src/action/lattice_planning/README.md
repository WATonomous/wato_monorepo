# Lattice Planning Module

Smooth, kinematically-feasible lattice path generation using cubic spiral optimization with map-aware corridor sampling and cost-based path selection.

## Overview

Generates drivable paths by:
1. Receiving ego vehicle pose and lattice lanelet corridor from world modeling
2. For the **ego lane** (including fork variants): collecting the raw centreline up to a velocity-scaled horizon and converting it directly to a path
3. For **adjacent lanes** (lane changes only): sampling terminal points at multiple lookahead horizons and fitting cubic spirals between the current pose and each terminal
4. Scoring all candidate paths (ego centrelines + lane-change spirals) through the cost function
5. Publishing the lowest-cost path for the tracking controller

**Current Status**: Functional spiral path generation with 3-DOF pose optimization. Integrates with behavior planner for lane preference and lanelet map for corridor constraints. Ego lane and fork variants are passed directly as centreline paths; cubic spirals are only generated for lane-change candidates.

## ROS Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `odom` | `nav_msgs/Odometry` | Ego vehicle odometry (position, orientation, velocity, curvature) |
| `lanelet_ahead` | `lanelet_msgs/LaneletAhead` | Lattice lanelet corridor with centerlines and topology |
| `execute_behaviour` | `behaviour_msgs/ExecuteBehaviour` | Preferred lanelet IDs from behavior planner |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Selected lowest-cost path as sequence of poses |
| `available_paths` | `lattice_planning_msgs/PathArray` | All candidate paths (ego centrelines + lane-change spirals) and their costs |

## Architecture

**Lifecycle node with separate algorithm core**:

- **lattice_planning_node**: ROS 2 lifecycle node handling subscriptions, publications, and corridor processing
- **lattice_planning_core**: Pure algorithm library for spiral generation, cost evaluation, and path selection
  - `generate_path()`: Damped Newton optimizer fitting cubic spirals k(s) = c0 + c1s + c2s² + c3s³
  - `path_cost_function()`: Evaluates curvature change penalties, lateral movement cost, and lane preference
  - `get_lowest_cost_path()`: Selects minimum-cost path from candidate set
  - `compute_jacobian_3dof()`: Finite-difference Jacobian for 3-DOF pose error (x, y, theta)
  - `centreline_to_path_points()`: Converts raw lanelet centreline points to `PathPoint` structs with heading and curvature via finite differences

## Algorithm Details

This algorithm was built from the following [paper](https://www.ri.cmu.edu/pub_files/2011/5/20100914_icra2011-mcnaughton.pdf), check it out to see the derivation of equations and more details about the following.

### Ego Lane Path Generation

Rather than generating a spiral toward the ego lane, the raw centreline is collected directly from the lanelet map and converted to a `PathPoint` sequence. This avoids unnecessary optimization on the current lane and provides a stable reference path:

- Centreline points ahead of the vehicle are accumulated up to a **velocity-scaled horizon**: `horizon = centreline_horizon × velocity × centreline_velocity_scale`
- If velocity is unavailable or zero, `centreline_horizon` is used as the fixed fallback
- Heading is computed via forward finite difference (backward at the last point)
- Curvature is computed as dθ/ds between consecutive points
- Each unique ego-lane sequence — including **fork variants** at road splits — produces a separate centreline path so the costmap can score and select between them

### Lane-Change Path Generation (Cubic Spirals)

Corridor terminal points are **only generated for adjacent lanes** (left/right of ego). Spiral paths are then fit from the current vehicle pose to each terminal:

- k(s) = c0 + c1s + c2s² + c3s³
- Optimization variables: intermediate curvatures k1, k2 (at 1/3 and 2/3 arc length), total arc length sf
- Solved via damped Newton's method with curvature clamping to vehicle limits
- Forward Euler integration discretizes spirals into waypoints

### Cost Function

All candidate paths — ego centrelines and lane-change spirals — are scored uniformly on:
- **Curvature change**: Penalizes exceeding max curvature change threshold (vehicle dynamics limits)
- **Lateral movement**: Weighted by absolute curvature (encourages straight paths)
- **Lane preference**: Fixed penalty for non-preferred lanes (from behavior planner)
- **Unknown occupancy**: Penalty for paths passing through unknown occupancy regions

### Corridor Sampling

- For the **ego lane**: collects centreline points up to the velocity-scaled horizon; one sequence per fork variant
- For **adjacent lanes**: samples terminal points at configurable lookahead distances (default: 10m, 17.5m, 25m); generates spiral path for each terminal
- Handles road forks by producing an independent ego-lane centreline path per fork branch, all of which are forwarded to the costmap

## Configuration

Parameters in `config/lattice_planning_params.yaml`:

**Corridor Sampling**:
- `num_lane_switch_horizons`: Number of lookahead distances for lane-change terminals (default: 3)
- `lane_switch_lookahead_distances`: Arc length sampling points for lane-change terminals in meters (default: [10.0, 17.5, 25.0])
- `centreline_horizon`: Fixed lookahead distance for ego-lane centreline in meters (default: 30.0); used as fallback when velocity is zero or unavailable
- `centreline_velocity_scale`: Scales ego-lane horizon by current velocity — `horizon = centreline_horizon × velocity × scale` (default: 0.2)

**Path Generation (Optimization)**:
- `max_iterations`: Newton solver max iterations (default: 20)
- `path_steps`: Spiral discretization steps (default: 35)
- `convergence_tolerance`: 3-DOF pose error threshold in m/rad (default: 0.15)
- `newton_damping`: Damping factor for Newton updates (default: 0.7)
- `max_step_size`: Maximum parameter update per iteration (default: 1.0)

**Cost Function**:
- `lateral_movement_weight`: Curvature penalty weight (default: 2.0)
- `physical_limits_weight`: Curvature change violation penalty (default: 4.0)
- `preferred_lane_cost`: Fixed cost for non-preferred lanes (default: 20.0)
- `unknown_occupancy_cost`: Penalty for paths in unknown occupancy regions (default: 50.0)
- `max_curvature_change`: Max curvature change threshold in rad/m (default: 0.1)

## Current Limitations

- **3-DOF optimization only**: Curvature at the lane-change terminal point is not enforced (optimizes x, y, theta but not kappa)
- **No obstacle avoidance**: Paths follow lanelet centrelines without dynamic object consideration
- **Single-threaded**: All candidate paths generated sequentially
- **No path smoothing post-processing**: Relies solely on cubic spiral smoothness for lane-change paths; ego centreline smoothness depends on map quality
- **Hard-coded corridor width**: Splitting behaviour maximises at a 3-lane corridor (left/ego/right)
- **Doesn't Use Strict Frenet Frame**: Stores all points as (x, y, theta, kappa) instead of a Frenet frame
- **No convergence diagnostics**: Returns empty path on failure without intermediate results
- **Ego centreline resolution is map-dependent**: Path density follows the lanelet map point spacing with no resampling

## Dependencies

- ROS 2 (tested on Humble)
- Eigen3 (matrix operations for optimization)
- `lanelet_msgs`, `behaviour_msgs` (custom message types)
- `tf2_geometry_msgs` (quaternion conversions)
