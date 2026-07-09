# Developing interfacing_bringup

## Launch Files

| File | Purpose |
|------|---------|
| `interfacing.launch.yaml` | Full stack: sensors + CAN/control + healthchecker |
| `interfacing_sensors.launch.yaml` | GPS, LiDARs, TF |
| `interfacing_can.launch.yaml` | Joystick, mux, PID, OSCC, CAN state estimator |

> Bag recording lives in `perception_bringup` (`perception_bag.launch.yaml`),
> since the cameras are NITROS nodes that run in the perception container.

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

## After Launching

1. **Check lifecycle nodes reached active state:**

   ```bash
   ros2 lifecycle get /ackermann_mux_node
   ros2 lifecycle get /joystick_node
   ros2 lifecycle get /vel_driven_feedforward_pid_node
   ros2 lifecycle get /can_state_estimator_node
   ros2 lifecycle get /oscc_mux_node
   # Each should print: active
   ```

2. **Check topic healthchecker** (if full stack launched):

```bash
   curl http://localhost:8080
   ```

All topics should show `"healthy"`. Any `"stale"` or `"no_publishers"` entries indicate a sensor or driver issue.

1. **Spot-check key topics:**

   ```bash
ros2 topic hz /ackermann                          # 50 Hz from mux
   ros2 topic hz /can_state_estimator/body_velocity  # CAN rate
   ros2 topic hz /lidar/all/points_merged            # ~10 Hz

   ```

## Definition of Good Result

| Component | Check | Expected |
|-----------|-------|----------|
| Lifecycle nodes | All `ros2 lifecycle get` | `active` |
| Healthchecker | `curl localhost:8080` | All topics `"healthy"` |
| CAN stack | `/ackermann` publishing | 50 Hz |
| Sensor stack | `/lidar/all/points_merged` publishing | ~10 Hz |
| Sensor stack | `/novatel/oem7/bestpos` publishing | ~5 Hz |

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
| FLIR Blackfly GigE | `flir-camera-driver` | [Camera bringup](../../perception/perception_bringup/docs/camera_bringup.md) |
| Velodyne VLP32C/VLP16 | `velodyne` | [LiDAR bringup](../sensor_interfacing/docs/lidar_bringup.md) |
