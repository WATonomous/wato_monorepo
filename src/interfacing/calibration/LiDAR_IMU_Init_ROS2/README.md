# LiDAR-IMU Extrinsic Calibration (LI-Init ROS2)

Based on [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init) by the HKU MaRS Lab.
ROS2 port by [morte2025](https://github.com/morte2025/LiDAR_IMU_Init_ROS2).
Additional modifications for Velodyne VLP-32C + Novatel OEM7 by our team.

> Zhu, F., Ren, Y., & Zhang, F. (2022). Robust real-time lidar-inertial initialization. *IROS 2022*, pp. 3948-3955.

## Overview

LI-Init calibrates the **extrinsic rotation and translation**, **temporal offset**, **IMU biases**, and **gravity vector** between a LiDAR and an IMU. No targets, prior maps, or initial guesses are required.

The output includes URDF-ready `xyz` and `rpy` values for both joint directions.

## Prerequisites

- ROS2 Jazzy (tested) or Humble
- Ceres Solver 2.2+
- PCL
- Eigen3

## Build

```bash
cd /path/to/workspace
colcon build --packages-select lidar_imu_init --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Configuration

Edit `config/velodyne.yaml` (or the appropriate config for your LiDAR):

```yaml
laser_mapping:
  ros__parameters:
    common:
        lid_topic:  "/your/lidar/topic"      # PointCloud2 topic
        imu_topic:  "/your/imu/topic"         # sensor_msgs/Imu topic (needs angular_vel + linear_accel)

    preprocess:
        lidar_type: 2                # 1: Livox, 2: Velodyne, 3: Ouster
        scan_line: 32                # Number of scan lines (e.g. 16, 32, 64, 128)
        blind: 2.0                   # Minimum range (meters)

    initialization:
        cut_frame_num: 3             # Sub-frames per scan (positive integer)
        orig_odom_freq: 20           # LiDAR scan frequency in Hz
        mean_acc_norm: 9.81          # ~9.81 for normal IMU, 1 for Livox built-in IMU
        online_refine_time: 20.0     # Seconds of online refinement after init
        data_accum_length: 60.0      # Data accumulation threshold for initialization

    mapping:
        filter_size_surf: 0.5        # Voxel filter for scan (0.05-0.15 indoor, 0.5 outdoor)
        filter_size_map: 0.5         # Voxel filter for map (0.15-0.25 indoor, 0.5 outdoor)
```

### Key parameter notes

- `cut_frame_num * orig_odom_freq` should be ~60 for mechanical LiDARs, ~50 for Livox.
- `data_accum_length`: Lower values (e.g. 60) work better for ground vehicles with limited pitch/roll excitation. Higher values require more rotational diversity.
- `filter_size_surf`: Must not be too small for dense point clouds (e.g. VLP-32C at ~29K points/frame). Values below 0.1 may cause VoxelGrid integer overflow.

## Running the Calibration

### Step 1: Launch the calibration node

```bash
source install/setup.bash
ros2 run lidar_imu_init li_init \
  --ros-args \
  -p point_filter_num:=3 \
  -p max_iteration:=5 \
  -p cube_side_length:=1000.0 \
  --params-file /path/to/install/lidar_imu_init/share/lidar_imu_init/config/velodyne.yaml
```

### Step 2: Play your bag (in a separate terminal)

```bash
ros2 bag play /path/to/your/bag.mcap \
  --topics /your/lidar/topic /your/imu/topic
```

### Step 3: Excite the sensors

The algorithm needs rotational excitation around all three axes to converge. Progress bars show rotation coverage for each axis.

**For handheld systems:** Rotate the sensor rig in all directions — roll, pitch, and yaw.

**For ground vehicles:** The algorithm will converge on yaw quickly from normal driving. Pitch and roll are harder to excite:
- Drive over bumps and curbs
- Accelerate and brake hard
- Make sharp turns at speed
- Figure-8 patterns help

**Tip:** Stay still for ~5 seconds at the start to build an initial map before moving.

### Step 4: Read the results

Once all three rotation axes reach 100%, initialization runs automatically, followed by online refinement. The final output includes:

```
[Final Result] Rotation LiDAR to IMU    =  <roll> <pitch> <yaw> deg
[Final Result] Translation LiDAR to IMU =  <x> <y> <z> m
[Final Result] Time Lag IMU to LiDAR    =  <seconds> s
[Final Result] Bias of Gyroscope        =  <x> <y> <z> rad/s
[Final Result] Bias of Accelerometer    =  <x> <y> <z> m/s^2
[Final Result] Gravity in World Frame   =  <x> <y> <z> m/s^2

[URDF] parent=lidar_link, child=imu_link
[URDF] xyz="<x> <y> <z>"
[URDF] rpy="<roll> <pitch> <yaw>"

[URDF] parent=imu_link, child=lidar_link
[URDF] xyz="<x> <y> <z>"
[URDF] rpy="<roll> <pitch> <yaw>"
```

## Interpreting the Output

### Coordinate transform (R_LI, T_LI)

The "Rotation/Translation LiDAR to IMU" is a **coordinate transform** — it converts points from the LiDAR frame to the IMU frame:

```
p_imu = R_LI * p_lidar + T_LI
```

The rotation is printed as ZYX Euler angles (roll, pitch, yaw) in degrees. The translation is in meters, expressed in the IMU frame.

### URDF values

The URDF output provides `xyz` (translation) and `rpy` (rotation in radians) for use in a `<joint><origin>` element. Both joint directions are printed — pick the one matching your URDF structure:

```xml
<joint name="lidar_imu_joint" type="fixed">
  <parent link="lidar_link"/>
  <child link="imu_link"/>
  <origin xyz="-0.570 -0.146 -0.456" rpy="0.016 0.005 -1.661"/>
</joint>
```

Note: the `rpy` (frame rotation) has the opposite sign from the printed Euler angles (coordinate transform). Both represent the same physical relationship.

### Accuracy notes

- **Rotation** is well-determined and consistent across runs. This is the most valuable output — it is difficult to measure by hand.
- **Translation** is less accurate for ground vehicles due to limited excitation. If the direction looks correct but the magnitude is off, consider measuring the translation physically with a tape measure and combining it with the calibrated rotation.
- **Time lag** is consistent and reliable.

## Troubleshooting

### "Leaf size is too small for the input dataset"
Increase `filter_size_surf` in the config (e.g. 0.5 for outdoor).

### Rotation progress stuck at 0% on X and Y axes
Ground vehicles have limited pitch/roll. Try driving over bumps, speed bumps, or rough terrain. Lowering `data_accum_length` (e.g. 60) also helps by reducing the excitation threshold.

### Node not receiving data
- Verify topic names match: `ros2 topic list` and `ros2 topic echo <topic> --once`
- Ensure IMU publishes `angular_velocity` and `linear_acceleration` (not just orientation)
- IMU angular velocity must be in **rad/s**, not deg/s

### Large false time lag detected at startup
If using Zenoh or a DDS that delivers small messages (IMU) faster than large messages (PointCloud2), the first-message time difference can be falsely large. This is handled by disabling HARD sync in `laserMapping.cpp` (`timediff_set_flg = true`).

## License

GPLv2. See the [original repository](https://github.com/hku-mars/LiDAR_IMU_Init) for details.

## Acknowledgments

- [HKU MaRS Lab](https://github.com/hku-mars) — original LI-Init, FAST-LIO2, ikd-Tree
- [Livox Technology](https://www.livoxtech.com/) — equipment support for the original project
