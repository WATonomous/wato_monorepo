# Developing sensor_interfacing

## Launch Files

| File | Purpose |
|------|---------|
| `launch/gps.launch.yaml` | Novatel OEM7 GPS + IMU driver |

Cameras and LiDARs are launched from `interfacing_bringup/interfacing_sensors.launch.yaml` using configs from this package.

## Config Files

| File | Description |
|------|-------------|
| `config/hikrobot/` | FLIR Blackfly GigE camera configurations (one per camera) |
| `config/novatel/` | Novatel OEM7 driver parameters |
| `config/velodyne/` | Velodyne driver parameters (one per LiDAR) |

## Adding a New Sensor

1. Add the driver package as a rosdep key in `package.xml`.
2. Create a config file under `config/<sensor_name>/`.
3. Add a launch entry in the appropriate `interfacing_bringup` launch file (`interfacing_sensors.launch.yaml`).
4. Add a bringup guide in `docs/` covering network setup, IP configuration, and driver verification.

## After Launching

Check each sensor stream is publishing at the expected rate:

```bash
# LiDARs (3 units)
ros2 topic hz /lidar_cc/velodyne_points
ros2 topic hz /lidar_ne/velodyne_points
ros2 topic hz /lidar_nw/velodyne_points

# GPS / IMU
ros2 topic hz /novatel/oem7/bestpos
ros2 topic hz /novatel/oem7/imu/data

# Cameras (NITROS pipeline; repeat for each of the 12 camera namespaces)
ros2 topic hz /camera_pano_nn/image_raw           # raw frames
ros2 topic hz /camera_pano_nn/image_rect          # GPU-rectified (Isaac ROS)
ros2 topic hz /camera_pano_nn/image_rect_h264     # NVENC H.264
```

## Definition of Good Result

| Sensor | Topic | Expected rate |
|--------|-------|--------------|
| Velodyne VLP32C / VLP16 | `/lidar_*/velodyne_points` | 10 Hz |
| Novatel OEM7 GPS | `/novatel/oem7/bestpos` | 5 Hz |
| Novatel OEM7 IMU | `/novatel/oem7/imu/data` | 100 Hz |
| FLIR Blackfly GigE cameras | `/camera_*/image_raw` | ~10 Hz |

If a sensor is not publishing:
- **LiDAR**: Check Ethernet connection and verify IP matches driver config (`config/velodyne/`)
- **GPS/IMU**: Check serial/Ethernet connection; verify Novatel driver is receiving BESTPOS corrections
- **Cameras**: Check GigE network adapter MTU (set to 9000 for jumbo frames); verify camera IP in driver config

## Driver Repositories

| Sensor | Repository |
|--------|-----------|
| Novatel OEM7 | https://github.com/novatel/novatel_oem7_driver/tree/humble |
| FLIR Blackfly GigE | https://github.com/ros-drivers/flir_camera_driver/tree/humble-devel |
| Velodyne | https://github.com/ros-drivers/velodyne/tree/ros2 |
