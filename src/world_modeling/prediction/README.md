# Prediction Module

Multi-modal trajectory prediction for vehicles, pedestrians, and cyclists using physics-based motion models with map-aware intent inference.

## Overview

Predicts future trajectories for tracked objects by:
1. Retreiving object type from Preception (vehicle/pedestrian/cyclist)
2. Querying HD map for current and possible future lanelets
3. Generating multiple trajectory hypotheses (different intents/paths)
4. Assigning probabilities to each hypothesis

**Current Status**: Skeleton with placeholder implementations. Runs standalone without map services.

## ROS Interface

### Subscribed Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/perception/detections_3D_tracked` | `vision_msgs/Detection3DArray` | Tracked objects |
| `/localization/pose` | `geometry_msgs/PoseStamped` | Ego vehicle pose |

### Published Topics

| Topic | Type | Description |
|-------|------|-------------|
| `/world_modeling/prediction/predicted_paths` | `wato_msgs/PredictionHypothesesArray` | Multi-modal predictions |

### Services Used

Map queries to `/world_modeling/lanelet/query/*` (placeholders until map services available)

## Architecture

**Component-based design for parallel team development**:

- **prediction_node**: Orchestrates pipeline, handles ROS communication
- **trajectory_predictor**: Generates hypotheses (person 1: pedestrian, person 2: vehicle, person 3: cyclist)
- **motion_models**: Physics-based propagation (bicycle model, constant velocity)
- **intent_classifier**: Assigns probabilities (shared by all)
- **map_interface**: HD map queries (currently placeholders)

Each component has single responsibility and can be tested independently.

## Quick Start

```bash
# Build
colcon build --packages-select prediction

# Run
ros2 launch prediction prediction.launch.py
```

## Team Tasks

**Person 1 - Pedestrian Prediction**:
- File: `src/trajectory_predictor.cpp` → `generatePedestrianHypotheses()`
- File: `src/motion_models.cpp` → Add noise to `ConstantVelocityModel`
- Use constant velocity with Gaussian noise, goal-directed behavior at crosswalks

**Person 2 - Vehicle Prediction**:
- File: `src/trajectory_predictor.cpp` → `generateVehicleHypotheses()`
- File: `src/motion_models.cpp` → `BicycleModel::generateTrajectory()`
- Implement bicycle kinematics with path following (pure pursuit or Stanley controller)

**Person 3 - Cyclist Prediction**:
- File: `src/trajectory_predictor.cpp` → `generateCyclistHypotheses()`
- Research cyclist behavior, implement hybrid model
- Use pedestrian model at crosswalks, vehicle model on roads
- **Critical**: Coordinate output format with Person 1 & 2

**Output Format** (all must match):

```cpp
struct TrajectoryHypothesis {
  std::vector<geometry_msgs::msg::Pose> waypoints;
  std::vector<double> timestamps;
  Intent intent;
  double probability;  // Set by classifier
};
```

## Configuration

Parameters in `config/params.yaml`:
- `prediction_horizon`: 5.0 seconds
- `prediction_time_step`: 0.1 seconds
- Vehicle/pedestrian/cyclist specific parameters

## Team Assignments

- **Girish**: Pedestrian prediction system (constant velocity model)
- **John**: Vehicle prediction system (bicycle kinematics)
- **Aruhant**: Cyclist prediction system (hybrid model)

See inline code comments marked with names for specific tasks.

## Current Limitations

- Using placeholder map data (synthetic lanelets and centerlines)
- Simple constant-velocity predictions
- Message types not yet defined in `wato_msgs`
- When map services available: uncomment service clients in `map_interface.cpp`
