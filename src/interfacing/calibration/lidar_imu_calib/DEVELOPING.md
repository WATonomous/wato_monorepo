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

### Single run

```bash
ros2 run lidar_imu_calib li_init \
  --ros-args --params-file install/lidar_imu_calib/share/lidar_imu_calib/config/velodyne.yaml
```

Then play a bag file or drive the vehicle:

```bash
ros2 bag play <bag_file> --topics /lidar_cc/velodyne_points /novatel/oem7/imu/data --rate 0.75
```

The node accumulates data until it has enough rotational excitation, then prints the result and exits.

### Iterative calibration (recommended for accuracy)

Run the iterative script to refine the estimate over many passes of the same bag:

```bash
cd scripts/
./iterative_calibration.sh 100   # 100 iterations
```

Each iteration uses the previous result as the initial guess. Results are saved to `scripts/calibration_results.csv`. The script prints a convergence summary at the end.

## After Launching

1. **Drive pattern** — drive in a figure-8 or repeated S-curves. The algorithm requires excitation in all three rotational axes (roll, pitch, yaw). Driving in a straight line provides yaw only and will not fully constrain the rotation.

2. **Watch the log** — the node prints progress. Wait for:

```
   Initialization result:
   Translation LiDAR to IMU (meter) = X Y Z
   Time Lag IMU to LiDAR (second)   = X
   ```

Then, after refinement:

```
   Refinement result:
   ```

Results are also written to `result/Initialization_result.txt` and printed as a ready-to-paste URDF `<origin>` block.

1. **Apply the result** — copy the printed `<origin xyz="..." rpy="..."/>` block into `eve_description/urdf/eve_kia_soul_ev.xacro` for the IMU joint.

## Definition of Good Result

**Single run:**
- The node reaches "Refinement result" (not just "Initialization result") — refinement requires enough motion to constrain translation in addition to rotation
- Point-to-plane residual `res_mean_last` (logged during refinement) should be < 0.05 m

**Iterative calibration (recommended):**

After running `iterative_calibration.sh 100`, the summary table shows mean and std dev across iterations (skipping 10 burn-in). A well-converged calibration satisfies:

| Quantity | Good | Poor |
|----------|------|------|
| Rotation std dev | < 0.5 deg | > 1.0 deg |
| Translation std dev | < 0.01 m | > 0.03 m |
| Time lag std dev | < 0.005 s | > 0.015 s |

If std devs are high, the bag does not have enough rotational excitation — record a new bag with more aggressive figure-8 maneuvers.

Once converged, use the "Recommended URDF (mean values)" line printed at the end of the script as the final transform.
