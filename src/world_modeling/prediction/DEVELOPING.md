# Prediction Module - Developer Guide

## File Structure Rationale

Component-based architecture enabling parallel development:

```
prediction_node.cpp       → ROS orchestrator (no algorithm logic)
trajectory_predictor.cpp  → Strategy pattern (select algorithm by object type)
motion_models.cpp         → Pure functions (physics-based state propagation)
intent_classifier.cpp     → Probability assignment (independent of trajectories)
map_interface.cpp         → Adapter for HD map services (currently placeholders)
```

**Why this structure?**
- **Separation of concerns**: Each file has one responsibility
- **Parallel work**: Person 1, 2, 3 modify different functions (no conflicts)
- **Testability**: Components tested independently
- **Extensibility**: Add new models/object types without changing existing code

## Implementation Details

### Placeholder Mode

Map services not required during development. Placeholders return:
- Deterministic lanelet IDs based on grid position
- Synthetic straight centerlines
- Mock crosswalk locations at grid intersections

**To switch to real map services**: Uncomment service client code in `MapInterface` constructor in `src/map_interface.cpp`.

### Output Format

All trajectory generators must return `std::vector<TrajectoryHypothesis>`:

```cpp
TrajectoryHypothesis hyp;
hyp.waypoints = /* std::vector<geometry_msgs::msg::Pose> */;
hyp.timestamps = /* std::vector<double> matching waypoints length */;
hyp.intent = Intent::CONTINUE_STRAIGHT;  // or other intent enum
hyp.probability = 0.0;  // Classifier sets this later
```

**Critical**: All three team members must use identical output format.

### Map Interface API

Available to all team members:

```cpp
// Get current lane
int64_t lanelet = map_interface_->findNearestLanelet(position);

// Get reachable future lanes
auto futures = map_interface_->getPossibleFutureLanelets(lanelet, depth);

// Get centerline for path following
LaneletInfo info = map_interface_->getLaneletById(lanelet);
std::vector<geometry_msgs::msg::Point> centerline = info.centerline;

// Check for crosswalk
bool crosswalk = map_interface_->isCrosswalkNearby(position, radius);
```

## Testing

```bash
# Build
colcon build --packages-select prediction

# Run with debug logs
ros2 launch prediction prediction.launch.py --ros-args --log-level debug

# Publish test detection
ros2 topic pub /perception/detections_3D_tracked vision_msgs/msg/Detection3DArray "..."
```

## Adding Dependencies

Follow monorepo guidelines in `/tmp/wato_monorepo/DEVELOPING.md`:
1. Prefer ROSdep dependencies (add to `package.xml`)
2. System packages only if no ROSdep key exists
3. Consider contributing to rosdistro for missing packages

## Future Enhancements

- Replace physics-based models with learned models
- Add ML-based intent classifier
- Support for additional object types (trucks, motorcycles)
- Integration with tracking history for velocity estimation
- Turn signal observation from CAN bus
