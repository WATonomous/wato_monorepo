# Pull Request: Prediction Module Skeleton

## Overview
Complete skeleton implementation for multi-modal trajectory prediction system. Ready for team members to implement their assigned components.

## What's Included

### Architecture
- **Component-based design** enabling parallel development
- **Prediction Node**: ROS orchestrator (no merge conflicts)
- **Trajectory Predictor**: Strategy pattern for different object types
- **Motion Models**: Physics-based kinematics (bicycle, constant velocity)
- **Intent Classifier**: Probability assignment (shared by all)
- **Map Interface**: HD map queries with placeholders

### Team Tasks
- **Girish**: Pedestrian prediction (`generatePedestrianHypotheses` + constant velocity with noise)
- **John**: Vehicle prediction (`generateVehicleHypotheses` + bicycle model path following)
- **Aruhant**: Cyclist prediction (`generateCyclistHypotheses` + hybrid model)

### Key Features
- ✅ Builds successfully with `colcon build --packages-select prediction`
- ✅ Runs standalone without map services (placeholder mode)
- ✅ Clear task markers in code (`// GIRISH TASK:`, `// JOHN TASK:`, `// ARUHANT TASK:`)
- ✅ Consistent output format enforced across all implementations
- ✅ Full ROS 2 integration (subscribers, publishers, parameters)
- ✅ Comprehensive documentation (README + DEVELOPING.md)

## File Structure
```
prediction/
├── README.md              # User-facing overview and quick start
├── DEVELOPING.md          # Technical details and architecture rationale
├── CMakeLists.txt         # Build configuration
├── package.xml            # ROS package manifest
├── config/
│   └── params.yaml        # Tunable parameters
├── include/prediction/    # Header files (interfaces)
│   ├── prediction_node.hpp
│   ├── trajectory_predictor.hpp
│   ├── motion_models.hpp
│   ├── intent_classifier.hpp
│   └── map_interface.hpp
├── src/                   # Implementation files
│   ├── prediction_node.cpp
│   ├── trajectory_predictor.cpp
│   ├── motion_models.cpp
│   ├── intent_classifier.cpp
│   └── map_interface.cpp
└── launch/
    └── prediction.launch.py
```

## Changes from Previous Version
- ❌ Removed old Kalman filter implementation
- ✅ Added complete prediction pipeline structure
- ✅ Added placeholder implementations (no external dependencies)
- ✅ Simplified documentation (removed redundant files)
- ✅ Fixed build issues (removed non-existent wato_msgs dependency)

## Documentation
- **README.md**: Quick overview, ROS interface, team tasks
- **DEVELOPING.md**: Architecture rationale, implementation details, testing

## Current Status
- **Buildable**: No compilation errors
- **Runnable**: Standalone mode with placeholders
- **Testable**: Can publish mock detections and observe behavior
- **Ready for implementation**: Clear TODOs for each team member

## Next Steps for Team
1. Each person implements their assigned function
2. Coordinate on output format (critical!)
3. Test with mock detections
4. When map services ready: uncomment service clients in `map_interface.cpp`
5. Define message types in `world_modeling_msgs`

## Testing
```bash
# Build
colcon build --packages-select prediction

# Run
ros2 launch prediction prediction.launch.py

# Test with mock detection
ros2 topic pub /perception/detections_3D_tracked vision_msgs/msg/Detection3DArray "..."
```

## Dependencies
- ROS 2 Humble
- vision_msgs
- geometry_msgs
- Eigen3
- No external map services required (placeholders)

## Notes
- Map interface uses placeholders (synthetic data) until services available
- Output format enforced through `TrajectoryHypothesis` struct
- All team members work in isolated functions (parallel development)
- Intent classifier shared by all (no modifications needed)

---

**Ready for review and team implementation!** 🚀
