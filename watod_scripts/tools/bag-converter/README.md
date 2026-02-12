# bag-converter

Offline ROS2 (rosbag2 MCAP) → ROS1 (`.bag`) converter used by `watod bag convert_ros1`.

## Why this exists

Most LiDAR–IMU calibration tools we want to use are ROS1-first. On Jazzy/Ubuntu 24.04, a live ROS1↔ROS2 bridge is usually not practical, so we convert a *single* `.mcap` chunk into a ROS1 `.bag`.

## Usage

From repo root:

- Show help:
  - `./watod bag convert_ros1 --help`

- Convert one MCAP chunk (recommended: include only standard message types):
  - `./watod bag convert_ros1 --src /bags/<chunk>.mcap --dst /bags/calib_inputs.bag \
      --include-topic /lidar_top/velodyne_points \
      --include-topic /novatel/oem7/imu/data_raw`

## Notes / limitations

- `rosbags` conversion currently does **not** handle split bags as a single logical recording; convert one `.mcap` at a time.
- ROS2→ROS1 conversion is most reliable for **default** message types (e.g., `sensor_msgs/msg/PointCloud2`, `sensor_msgs/msg/Imu`).
- For best LiDAR–IMU time offset calibration, `velodyne_packets` can be better than `velodyne_points`, but it may require message definitions to be present in the MCAP.
