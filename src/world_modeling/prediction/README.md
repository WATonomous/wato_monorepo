# Prediction Module

Multi-modal trajectory prediction for vehicles, pedestrians, and cyclists using physics-based motion models with lanelet-aware intent inference.

## Overview

Predicts future trajectories for tracked objects by:
1. Retrieving object type from Perception (vehicle/pedestrian/cyclist)
2. Querying HD map for reachable lanelets around the object
3. Generating multiple trajectory hypotheses using motion models
4. Assigning probabilities to each hypothesis with temporal smoothing

**Current Status**: Fully implemented with lanelet-aware prediction, per-vehicle caching, and async service queries.

## ROS Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/perception/detections_3D_tracked` | `vision_msgs/Detection3DArray` | Tracked objects from perception |
| `/localization/pose` | `geometry_msgs/PoseStamped` | Ego vehicle pose for reference frame |
| `/world_modeling/lanelet_ahead` | `lanelet_msgs/LaneletAhead` | Ego-relative reachable lanelets |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/world_modeling/world_object_seeds` | `world_model_msgs/WorldObjectArray` | Predicted objects with trajectory hypotheses |

### Services Used

| Service | Type | Description |
|---------|------|-------------|
| `/world_modeling/get_lanelet_ahead` | `lanelet_msgs/srv/GetLaneletAhead` | Query lanelets around a vehicle position (async, per-vehicle cached) |

## Architecture

**Modular component design**:

- **PredictionNode**: Lifecycle management, ROS communication, temporal smoothing
  - Subscribes to detections, ego pose, ego-relative lanelets
  - Manages async per-vehicle lanelet service requests
  - Applies confidence smoothing to reduce frame-to-frame flicker
  - Publishes world objects with trajectory hypotheses

- **TrajectoryPredictor**: Hypothesis generation with lanelet awareness
  - **generateHypotheses()**: Routes to type-specific generators
  - **generateLaneletVehicleHypotheses()**: Path-following hypotheses (left/right/straight)
  - **generateGeometricVehicleHypotheses()**: Fallback when no lanelet data
  - **generatePedestrianHypotheses()**: Constant velocity with intent variation
  - **generateCyclistHypotheses()**: Hybrid vehicle/pedestrian behavior
  - Per-vehicle lanelet caching with invalidation distance
  - Speed estimation from position history

- **MotionModels**: Physics-based trajectory propagation
  - **BicycleModel**: Kinematic bicycle model for vehicle trajectories
  - **ConstantVelocityModel**: Simple velocity propagation for pedestrians

- **IntentClassifier**: Probability assignment to hypotheses
  - Geometric scoring (heading alignment, lanelet match quality)
  - Maneuver priors and inertia
  - Trajectory smoothness penalties

Each component has single responsibility and clear interfaces.

## Quick Start

```bash
# Build prediction module and dependencies
colcon build --packages-select prediction world_model

# Run prediction node with world model
ros2 launch prediction prediction.launch.py
```

## Key Features

### Lanelet-Aware Prediction
- Queries reachable lanelets around detected vehicles via `get_lanelet_ahead` service
- Per-vehicle caching prevents redundant service requests within 5m movement threshold
- Falls back to geometric prediction when lanelet data unavailable

### Temporal Smoothing
- Confidence smoothing (α-filter) reduces hypothesis flickering between frames
- Matches hypotheses by intent and endpoint location (6m threshold)
- Timeout removes stale object state after 5 seconds

### Async Service Queries
- Non-blocking per-vehicle lanelet queries using ROS2 async service clients
- Limits concurrent requests (max 8 pending) to prevent service overload
- Maintains per-vehicle cache keyed by detection ID

### Speed Estimation
- Tracks position history per object for velocity estimation
- Falls back to bounding box length heuristic when history unavailable
- Used to parameterize motion models

## Configuration

Parameters in `config/params.yaml`:
- `prediction_horizon`: 3.0 seconds (prediction time window)
- `prediction_time_step`: 0.2 seconds (discretization step)
- `confidence_smoothing_alpha`: 0.35 (smoothing factor 0-1)
- `confidence_match_distance_m`: 6.0 (hypothesis matching threshold)
- `confidence_state_timeout_s`: 5.0 (object memory timeout)

## Future Enhancements

- Learned motion models replacing physics-based models
- ML-based intent classifier leveraging trajectory history
- Support for additional object types (trucks, motorcycles, buses)
- Vehicle turn signal observation from CAN bus
- Integration with tracking system for full motion history
- Cross-frame identity consistency with tracking module
- Uncertainty quantification and covariance estimation
- Interaction-aware prediction (multi-agent coordination)
