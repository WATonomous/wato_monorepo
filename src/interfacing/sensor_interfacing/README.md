# sensor_interfacing

Launch files and configuration for the physical sensors on Eve — GPS/IMU, cameras, and LiDARs.

## Overview

This package does not contain any custom ROS nodes. It holds launch files, driver configurations, and bringup guides for the third-party sensor drivers used on the vehicle. The actual driver packages are pulled in as rosdep dependencies.

## Sensors

| Sensor | Driver | Interface |
|--------|--------|-----------|
| Novatel OEM7 (GPS + IMU) | `novatel_oem7_driver` | Ethernet |
| FLIR Blackfly GigE (×12 cameras) | `flir_camera_driver` | GigE |
| Velodyne VLP32C / VLP16 (×3 LiDARs) | `velodyne` | Ethernet |

## Bringup Guides

Step-by-step setup instructions for each sensor are in `docs/`:

- [GPS bringup](docs/gps_bringup.md)
- [Camera bringup](docs/camera_bringup.md)
- [LiDAR bringup](docs/lidar_bringup.md)
