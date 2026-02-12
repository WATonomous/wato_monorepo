# li-init

Offline ROS1 (Noetic) runner for HKU-MARS LI-Init (LiDAR–IMU extrinsics + time offset initializer).

This tool is intended to be invoked via `./watod bag li_init ...`.

## What it does

- Starts a ROS1 master inside the container
- Launches LI-Init against a ROS1 `.bag` you produced via `watod bag convert_ros1`
- Plays the bag
- Waits until LI-Init writes `Initialization_result.txt`
- Copies the result into your mounted `/bags` directory

## Usage

```bash
./watod bag li_init \
  --bag /bags/calib_inputs.bag \
  --lidar-topic /lidar_top/velodyne_points \
  --imu-topic /novatel/oem7/imu/data_raw \
  --out /bags/li_init_result.txt
```

## Notes

- LI-Init expects pointcloud timestamps/fields suitable for deskewing. For Velodyne, your PointCloud2 should contain per-point time and ring fields (typical for `velodyne_pointcloud`).
- The output contains:
  - Rotation/translation from LiDAR to IMU
  - Time lag IMU to LiDAR (seconds)
  - IMU biases and gravity estimate
