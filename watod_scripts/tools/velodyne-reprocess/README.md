# Velodyne reprocess tool (ROS2)

This tool regenerates a `sensor_msgs/PointCloud2` from `velodyne_msgs/VelodyneScan` packets so the cloud has **valid per-point timing** (the `time` field should vary within a scan).

This is specifically intended for the case where an existing `velodyne_points` topic contains a `time` field that is constant (often all zeros), which can crash or break offline LiDAR–IMU initialization tools like LI-Init.

## Usage (via watod)

```bash
./watod bag packets_to_points \
  --src <ros2_mcap_dir_or_file_under_bags> \
  --dst <output_bag_dir_name> \
  --packets-topic /lidar_top/velodyne_packets \
  --points-topic /lidar_top/velodyne_points_timed \
  --imu-topic /novatel/oem7/imu/data_raw
```

By default, it uses `VLP32C` configs and `VeloView-VLP-32C.yaml` calibration from the installed `velodyne_pointcloud` package.

## Output

A new MCAP bag directory is created under `bags/` containing only:
- the regenerated pointcloud topic
- the IMU topic (passed through from playback)

You can then run the existing offline conversion + LI-Init on that output bag.
