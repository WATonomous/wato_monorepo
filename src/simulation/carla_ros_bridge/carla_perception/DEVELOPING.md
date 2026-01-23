# Developing carla_perception

## Design Rationale

Sensors are spawned in CARLA rather than just reading existing sensors because:
1. We control exact sensor placement via TF
2. We control sensor parameters (resolution, FOV, etc.)
3. Sensors are destroyed on deactivate, preventing resource leaks across scenario switches

Sensor positions come from TF (published by robot_state_publisher from URDF) so the sensor configuration is defined in one place.

## Architecture

Each sensor publisher node:
1. On configure: connects to CARLA, finds ego vehicle, creates ROS publishers
2. On activate: looks up sensor transforms from TF, spawns sensors in CARLA, registers callbacks
3. On deactivate: destroys sensors
4. On cleanup: destroys publishers, releases CARLA connection

Sensor data arrives via CARLA callbacks which run in CARLA's thread. Keep callback processing minimal.

## Multi-Sensor Support

Sensors are configured via list parameters (`camera_names`, `lidar_names`). For each name, the node reads namespaced parameters (e.g., `front_camera.image_width`).

This pattern allows spawning multiple sensors of the same type with different configurations.

## Coordinate Transforms

CARLA uses left-handed coordinates (X-forward, Y-right, Z-up). ROS uses right-handed (X-forward, Y-left, Z-up). The Y axis is flipped when converting positions and rotations.

Camera optical frames have Z-forward, X-right, Y-down. The `optical_frame` parameter indicates whether the configured frame_id follows this convention.

## Adding a New Sensor Type

1. Create a new lifecycle node
2. Follow the same configure/activate/deactivate/cleanup pattern
3. Use `_declare_if_not_exists` for per-sensor parameters
4. Convert CARLA sensor data to appropriate ROS message type
5. Add entry point in setup.cfg
