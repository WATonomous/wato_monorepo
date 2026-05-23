# Developing interfacing_bringup

## Launch Files

| File | Purpose |
|------|---------|
| `interfacing.launch.yaml` | Full stack: sensors + CAN/control + healthchecker |
| `interfacing_sensors.launch.yaml` | GPS, LiDARs, cameras only |
| `interfacing_can.launch.yaml` | Joystick, mux, PID, OSCC, CAN state estimator |
| `interfacing_bag.launch.yaml` | Bag recording launch for various sensor configs |

### interfacing_can.launch.yaml arguments

| Argument | Type | Default | Description |
|----------|------|---------|-------------|
| `config_file` | string | `config/interfacing.yaml` | Path to unified parameter file |
| `autostart` | bool | `true` | Auto-transition lifecycle nodes to active |
| `test_velocity` | bool | `false` | Launch the velocity trapezoid dummy node |
| `test_steering` | bool | `false` | Launch the square wave steering dummy node |

## Build & Launch

```bash
colcon build --packages-select interfacing_bringup

# Full interfacing stack
ros2 launch interfacing_bringup interfacing.launch.yaml

# CAN/control stack only
ros2 launch interfacing_bringup interfacing_can.launch.yaml

# Sensors only
ros2 launch interfacing_bringup interfacing_sensors.launch.yaml
```

## Config File Structure

All node parameters live in `config/interfacing.yaml`. Each top-level key matches a node name. Parameters for individual packages are documented in their own DEVELOPING.md files:

- `ackermann_mux` → [ackermann_mux/DEVELOPING.md](../ackermann_mux/DEVELOPING.md)
- `oscc_mux` → [oscc_mux/DEVELOPING.md](../oscc_mux/DEVELOPING.md)
- `joystick_node` → [joystick_interfacing/DEVELOPING.md](../joystick_interfacing/DEVELOPING.md)
- `vel_driven_feedforward_pid_node` → [pid_control/DEVELOPING.md](../pid_control/DEVELOPING.md)
- `can_state_estimator_node` → [can_state_estimator/DEVELOPING.md](../can_state_estimator/DEVELOPING.md)
- `oscc_interfacing_node` → [oscc_interfacing/DEVELOPING.md](../oscc_interfacing/DEVELOPING.md)

## Adding a New Node to the Stack

1. Add the package to `package.xml` as an exec dependency.
2. Add a node entry in the appropriate subsystem launch file.
3. Add its parameters under a matching key in `config/interfacing.yaml`.
4. If it is a lifecycle node, add it to the `wato_lifecycle_manager` node list in the launch file.

## Sensor Driver Setup

Third-party drivers are available as rosdep keys — no source build needed:

| Sensor | rosdep key | Setup guide |
|--------|-----------|-------------|
| Novatel OEM7 | `novatel-oem7-driver` | [GPS bringup](../sensor_interfacing/docs/gps_bringup.md) |
| FLIR Blackfly GigE | `flir-camera-driver` | [Camera bringup](../sensor_interfacing/docs/camera_bringup.md) |
| Velodyne VLP32C/VLP16 | `velodyne` | [LiDAR bringup](../sensor_interfacing/docs/lidar_bringup.md) |
