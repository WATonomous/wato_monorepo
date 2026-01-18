# Developing carla_localization

## Design Rationale

Ground truth localization from CARLA provides a known-good pose for:
1. Testing perception/planning without localization errors
2. Comparing estimated pose against ground truth
3. Development before real localization is available

The TF tree follows REP 105 conventions with map -> odom -> base_link.

## Architecture

A timer publishes TF at `publish_rate` Hz. Each tick:
1. Get ego vehicle transform from CARLA
2. Convert from CARLA coordinates to ROS coordinates
3. Publish map -> odom (identity) and odom -> base_link transforms

## Coordinate System Conversion

CARLA (left-handed): X-forward, Y-right, Z-up
ROS (right-handed): X-forward, Y-left, Z-up

Position: Y is negated
Rotation: pitch and yaw are negated (roll stays the same)

The `carla_common` package provides `euler_to_quaternion` for this conversion.

## Why map -> odom is Identity

In real robots, map -> odom corrects for odometry drift. In simulation there's no drift, so it's always identity. This keeps the TF tree structure consistent with real robot configurations.

## Extension: Additional Frames

To publish additional frames (e.g., sensor frames from CARLA):
1. Get the relative transform from CARLA
2. Apply coordinate conversion
3. Broadcast as child of base_link

However, static sensor frames are better defined in URDF and published by robot_state_publisher.
