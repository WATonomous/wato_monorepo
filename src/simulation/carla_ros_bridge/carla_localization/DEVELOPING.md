# Developing carla_localization

## Architecture

The localization node follows the standard lifecycle node pattern and publishes TF at a configurable rate.

## Coordinate Transformation

CARLA (left-handed) to ROS (right-handed):

```python
# Position
x = carla_transform.location.x
y = -carla_transform.location.y  # Flip Y
z = carla_transform.location.z

# Rotation (degrees to radians, with sign flips)
roll = math.radians(carla_transform.rotation.roll)
pitch = -math.radians(carla_transform.rotation.pitch)
yaw = -math.radians(carla_transform.rotation.yaw)
```

## TF Tree Structure

```
map
 └── odom (identity transform - no drift in simulation)
      └── base_link (vehicle pose)
```

The `map → odom` transform is identity because simulation has no odometry drift.

## Euler to Quaternion

Uses ZYX (yaw-pitch-roll) convention:
```python
def euler_to_quaternion(roll, pitch, yaw):
    # Standard ZYX quaternion conversion
```

## Extension Points

To add additional transforms (e.g., sensor frames):
1. Get sensor transforms relative to vehicle from CARLA
2. Apply coordinate conversion
3. Broadcast as child of `base_link`
