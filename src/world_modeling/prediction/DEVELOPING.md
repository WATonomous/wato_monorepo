# Prediction Module - Developer Guide

## File Structure

### Headers (include/prediction/)
- **prediction_node.hpp**: Lifecycle ROS2 node orchestrating the prediction pipeline
- **trajectory_predictor.hpp**: Core hypothesis generation with lanelet awareness
- **motion_models.hpp**: Physics-based kinematic models (bicycle, constant velocity)
- **intent_classifier.hpp**: Probability assignment to trajectory hypotheses

### Implementation (src/)
- **prediction_node.cpp**: Lifecycle callbacks, ROS communication, temporal smoothing
- **trajectory_predictor.cpp**: Object-type-specific hypothesis generators
- **motion_models.cpp**: Motion model implementations
- **intent_classifier.cpp**: Scoring and probability computation

## Core Concepts

### TrajectoryHypothesis Structure

```cpp
struct TrajectoryHypothesis {
  std_msgs::msg::Header header;           // Timestamp and frame
  std::vector<geometry_msgs::msg::PoseStamped> poses;  // Waypoint sequence
  Intent intent;                           // Maneuver type
  double probability;                      // Assigned by classifier
};
```

Each hypothesis represents one possible future path of constant intent.

### Intent Enumeration

```cpp
enum class Intent {
  CONTINUE_STRAIGHT,    // Maintain current lane
  TURN_LEFT,            // Follow left-turning lanelet
  TURN_RIGHT,           // Follow right-turning lanelet
  LANE_CHANGE_LEFT,     // Cross into left lanelet
  LANE_CHANGE_RIGHT,    // Cross into right lanelet
  STOP,                 // Decelerating to stop
  UNKNOWN               // Fallback for ambiguous cases
};
```

### Lanelet-Aware Prediction

**Process**:
1. Query `GetLaneletAhead` service for lanelets reachable from object position
2. For each reachable lanelet, generate trajectory following its centerline
3. Extract forward path with smoothness checking
4. Propagate trajectory using motion model with distance normalization
5. Score based on lanelet match quality and path geometry

**Caching**:
- Per-vehicle cache indexed by detection ID
- Invalidated when vehicle moves >5m from cached position
- Prevents redundant service queries in same prediction cycle

**Async Requests**:
- Fire-and-forget async service calls for per-vehicle queries
- Limits concurrent requests (max 8) via `pending_vehicle_requests_` set
- Results populated into `vehicle_lanelet_cache_` when available

### Fallback Geometric Prediction

When lanelet data unavailable:
1. Estimate current heading from velocity or detection orientation
2. Generate hypotheses in cardinal directions
3. Propagate using constant velocity model
4. Score based on heading difference only

### Temporal Smoothing

**Confidence Smoothing**:
- Exponential α-filter to reduce flickering
- Matches current hypotheses to previous frame by intent + endpoint location
- Tolerance: 6m endpoint distance, matching intent
- Interpolates confidence across frames

**State Pruning**:
- Objects not seen for 5 seconds removed from confidence history
- Prevents memory growth with stale detections

### Speed Estimation

**Multi-source**:
1. **Position History**: Track detection position over frames
   - Compute velocity from displacement between observations
   - More reliable than observation noise when history available
2. **Bounding Box Heuristic**: Use bbox length as speed proxy
   - Fallback when position history empty (first frame)
   - Provides minimum speed baseline

## Implementation Patterns

### Object Type Routing

```cpp
ObjectType object_type = classifyObjectType(detection);
switch(object_type) {
  case ObjectType::VEHICLE:
    return generateLaneletVehicleHypotheses(detection, speed); // lanelet-aware
  case ObjectType::PEDESTRIAN:
    return generatePedestrianHypotheses(detection);
  case ObjectType::CYCLIST:
    return generateCyclistHypotheses(detection);
  // ...
}
```

### Vehicle Hypothesis Generation
For each reachable lanelet:
1. Extract centerline as reference path
2. Adapt path length based on estimated speed and horizon
3. Propagate bicycle model with lane-following controller
4. Return hypothesis with matching intent and probability=0 (classifier sets)

### Pedestrian Context-Aware Prediction

Pedestrians use a spatial query (`GetNearbyLanelets`) that bypasses the vehicle routing graph
and returns ALL lanelet types (road, crosswalk, parking). A context classifier dispatches to
context-specific hypothesis generators:

| Context | Trigger | Hypotheses |
|---------|---------|------------|
| **Crosswalk** | Nearest lanelet is `crosswalk` type | Forward/reverse along crosswalk centerline + stop (2-3) |
| **Road-adjacent** | Nearest lanelet is `road` or `intersection` | Forward walk + perpendicular road-crossing (jaywalking) + stop (3) |
| **Parking / Open area** | Nearest is `parking`, or no lanelets nearby | Multi-directional fan (5 directions) + stop (6) |

**Priority**: crosswalk > parking > road > open area.

**Fallback chain**: Service unavailable → open-area fan. No nearby lanelets → open-area fan.
Context generator returns empty → open-area fan. Every path produces valid hypotheses.

**Caching**: Per-pedestrian cache (`pedestrian_lanelet_cache_`) with separate mutex.
Invalidated when pedestrian moves >3m. Separate from vehicle cache (different data types).

### Service Query Pattern

```cpp
// Vehicle lanelet queries (routing-graph-based)
auto callback = [this](rclcpp::Client<GetLaneletAhead>::SharedFuture future) {
  auto response = future.get();
  trajectory_predictor_->updateVehicleLaneletCache(
      vehicle_id, response->lanelet_ahead, x, y);
};
lanelet_ahead_client_->async_send_request(request, callback);

// Pedestrian lanelet queries (spatial, non-routing)
auto ped_callback = [this](rclcpp::Client<GetNearbyLanelets>::SharedFuture future) {
  auto response = future.get();
  trajectory_predictor_->updatePedestrianLaneletCache(
      ped_id, response->lanelets, px, py);
};
nearby_lanelets_client_->async_send_request(request, ped_callback);
```

## Testing

### Unit Testing
Test individual components without ROS:

```cpp
// trajectory_predictor with mock lanelet data
auto predictor = std::make_unique<TrajectoryPredictor>(node, 3.0, 0.2);
predictor->setTemporaryLaneletData(mock_lanelet);
auto hyps = predictor->generateHypotheses(detection);
// Verify trajectory shape, intent, waypoint count
```

### Integration Testing

```bash
# Build with debug symbols
colcon build --packages-select prediction --cmake-args -DCMAKE_BUILD_TYPE=Debug

# Run with log output
ros2 launch prediction prediction.launch.py --ros-args --log-level debug

# Publish test detection in another terminal
ros2 topic pub /perception/detections_3D_tracked vision_msgs/msg/Detection3DArray "{
  detections: [{
    bbox: {center: {position: {x: 10.0, y: 0.0, z: 0.0}}, ...},
    results: [{hypothesis: {class_name: 'vehicle'}}]
  }]
}"

# Monitor predictions
ros2 topic echo /world_modeling/world_object_seeds
```

## Extending the Module

### Adding a New Object Type
1. Add enum value to `ObjectType` in trajectory_predictor.hpp
2. Add `classifyObjectType()` condition for detection classification
3. Implement `generate<Type>Hypotheses()` method returning `std::vector<TrajectoryHypothesis>`
4. Call from `generateHypotheses()` switch statement

### Improving Intent Classifier
Modify `IntentClassifier::computeGeometricScore()`:
- Add heading alignment penalty
- Weight lanelet match quality by path curvature
- Incorporate lateral velocity for lane-change detection

### Tuning Motion Models
Edit `motion_models.cpp`:
- Bicycle model: Adjust look-ahead distance, speed scaling
- Constant velocity: Add Gaussian process noise for pedestrians
- Both: Modify waypoint spacing for fidelity/performance tradeoff

## Known Limitations & TODO

- **No velocity scaling in bicycle model**: All trajectories use same arc radius
- **Pedestrian model**: Context-aware but no goal prediction or obstacle avoidance
- **No loop detection**: Lanelet queries can return cycles (e.g., roundabouts)
- **No turn signal integration**: All vehicle hypotheses equally likely
- **Limited object type classification**: Uses only bounding box aspect ratio
