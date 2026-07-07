# Developing sensor_interfacing

## Launch Files

| File | Purpose |
|------|---------|
| `launch/gps.launch.yaml` | Novatel OEM7 GPS + IMU driver |

LiDARs are launched from `interfacing_bringup/interfacing_sensors.launch.yaml` using configs from this package. The Hikrobot cameras are NITROS nodes that run in the perception container; their launch files and configs live in `perception_bringup`.

## Config Files

| File | Description |
|------|-------------|
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
```

## Definition of Good Result

| Sensor | Topic | Expected rate |
|--------|-------|--------------|
| Velodyne VLP32C / VLP16 | `/lidar_*/velodyne_points` | 10 Hz |
| Novatel OEM7 GPS | `/novatel/oem7/bestpos` | 5 Hz |
| Novatel OEM7 IMU | `/novatel/oem7/imu/data` | 100 Hz |

If a sensor is not publishing:
- **LiDAR**: Check Ethernet connection and verify IP matches driver config (`config/velodyne/`)
- **GPS/IMU**: Check serial/Ethernet connection; verify Novatel driver is receiving BESTPOS corrections

## Driver Repositories

| Sensor | Repository |
|--------|-----------|
| Novatel OEM7 | https://github.com/novatel/novatel_oem7_driver/tree/humble |
| Velodyne | https://github.com/ros-drivers/velodyne/tree/ros2 |
