# Developing lidar_imu_calib

## Messages

### Pose6D.msg

Preintegrated LiDAR state between two keyframes.

| Field | Type | Description |
|-------|------|-------------|
| `offset_time` | `float32` | Time offset from frame start |
| `acc` | `float32[3]` | Acceleration vector |
| `gyr` | `float32[3]` | Angular velocity vector |
| `vel` | `float32[3]` | Velocity |
| `pos` | `float32[3]` | Position |
| `rot` | `float32[9]` | Rotation matrix (row-major) |

### States.msg

Estimated state at the end of a LiDAR frame.

| Field | Type | Description |
|-------|------|-------------|
| `rot` | `float32[9]` | Rotation matrix |
| `pos` | `float32[3]` | Position |
| `vel` | `float32[3]` | Velocity |
| `bg` | `float32[3]` | Gyroscope bias |
| `ba` | `float32[3]` | Accelerometer bias |
| `grav` | `float32[3]` | Gravity vector |
| `cov` | `float32[23]` | State covariance |

## Build

```bash
colcon build --packages-select lidar_imu_calib
```

## Dependencies

- Ceres Solver
- PCL
- ikd-Tree (included under `include/ikd-Tree/`)

## Usage

Drive the vehicle along a path with sufficient rotational excitation. The algorithm requires variation in all three rotational axes to fully constrain the extrinsic solution. Once converged, record the output transform and update the URDF in `eve_description`.
