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

- LI-Init expects pointcloud fields suitable for deskewing. For Velodyne, your `sensor_msgs/PointCloud2` should contain:
  - `ring`
  - a per-point `time` field that **varies within a scan** (not all drivers populate this correctly)
- If `time` is present but constant (often all zeros), LI-Init may compute a bad lag and can crash. In that case, generate a timed pointcloud from packets first:

  ```bash
  ./watod bag packets_to_points \
    --src <your_ros2_mcap_under_bags> \
    --dst <output_bag_dir_name> \
    --packets-topic /lidar_top/velodyne_packets \
    --points-topic /lidar_top/velodyne_points_timed \
    --imu-topic /novatel/oem7/imu/data_raw
  ```

  Then run `watod bag convert_ros1` on the new output and run LI-Init using `/lidar_top/velodyne_points_timed`.
- The output contains:
  - Rotation/translation from LiDAR to IMU
  - Time lag IMU to LiDAR (seconds)
  - IMU biases and gravity estimate
- LI-Init is a ROS1 repo; keep it under `watod_scripts/tools/` (containers) rather than `src/interfacing/calibration/` (ROS2/colcon).
