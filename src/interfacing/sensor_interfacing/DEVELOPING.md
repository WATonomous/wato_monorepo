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

## Driver Repositories

| Sensor | Repository |
|--------|-----------|
| Novatel OEM7 | https://github.com/novatel/novatel_oem7_driver/tree/humble |
| FLIR Blackfly GigE | https://github.com/ros-drivers/flir_camera_driver/tree/humble-devel |
| Velodyne | https://github.com/ros-drivers/velodyne/tree/ros2 |
