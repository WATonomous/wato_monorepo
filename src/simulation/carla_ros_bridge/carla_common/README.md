# carla_common

Shared utilities for CARLA ROS bridge packages.

## Functions

**Coordinate conversion (CARLA left-handed to ROS right-handed):**
- `carla_to_ros_position(x, y, z)` - Negates Y axis
- `carla_to_ros_rotation(roll_deg, pitch_deg, yaw_deg)` - Converts degrees to radians, negates pitch/yaw

**Quaternion conversion:**
- `euler_to_quaternion(roll, pitch, yaw)` - Euler angles (radians) to quaternion (x, y, z, w)
- `quaternion_to_euler(x, y, z, w)` - Quaternion to Euler angles (radians)

**CARLA client utilities:**
- `connect_carla(host, port, timeout)` - Connect to CARLA server, returns `carla.Client`
- `find_ego_vehicle(world, role_name)` - Find vehicle by role_name attribute, returns vehicle or None

## Usage

```python
from carla_common import (
    connect_carla,
    find_ego_vehicle,
    carla_to_ros_position,
    carla_to_ros_rotation,
    euler_to_quaternion,
)

# Connect and find ego
client = connect_carla(host, port, timeout)
world = client.get_world()
vehicle = find_ego_vehicle(world, "ego_vehicle")

# Convert CARLA transform to ROS
transform = vehicle.get_transform()
x, y, z = carla_to_ros_position(
    transform.location.x,
    transform.location.y,
    transform.location.z,
)
roll, pitch, yaw = carla_to_ros_rotation(
    transform.rotation.roll,
    transform.rotation.pitch,
    transform.rotation.yaw,
)
qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)
```
