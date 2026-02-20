# Interfacing Bringup

Bringup package used to launch drivers for Eve.

## Current Drivers

### Novatel GPS Driver
**Link to GitHub Repo:** https://github.com/novatel/novatel_oem7_driver/tree/humble
**Usage Pattern:** This driver is available as a rosdep key, usage of this driver consists of depending on the released version in `package.xml` and launching.

**More documentation on setup:** [GPS Bringup](../sensor_interfacing/docs/gps_bringup.md)

### Teledyne Blackfly GigE Driver
**Link to GitHub Repo:** https://github.com/ros-drivers/flir_camera_driver/tree/humble-devel
**Usage Pattern:** This driver is available as a rosdep key, usage of this driver consists of depending on the released version in `package.xml` and launching.

**More documentation on setup:** [Camera Bringup](../sensor_interfacing/docs/camera_bringup.md)

### Velodyne VLP32C/VLP16 Drivers
**Link to Github Repo:** https://github.com/ros-drivers/velodyne/tree/ros2
**Usage Pattern** This driver is available as a rosdep key, usage of this driver consists of depending on the released version in `package.xml` and launching.

**More documentation on setup:** [LiDAR Bringup](../sensor_interfacing/docs/lidar_bringup.md)
