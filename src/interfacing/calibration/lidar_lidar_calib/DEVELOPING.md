# Developing lidar_lidar_calib

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `target` (configurable) | `sensor_msgs/PointCloud2` | Reference cloud (center LiDAR) |
| `source` (configurable) | `sensor_msgs/PointCloud2` | Cloud to align (NE or NW LiDAR) |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `calibrated_cloud` | `sensor_msgs/PointCloud2` | Source cloud transformed into target frame (for visual verification) |

## Configuration

Config files are in `config/`. Key parameters:

| Parameter | Type | Description |
|-----------|------|-------------|
| `initial_guess` | float[6] | Initial transform `[x, y, z, roll, pitch, yaw]` from physical measurement |
| `max_correspondence_distance` | double | GICP max point-pair distance (metres); reduce as convergence improves |
| `target_topic` | string | Topic for the reference (center) cloud |
| `source_topic` | string | Topic for the cloud to align |

## Build & Launch

```bash
colcon build --packages-select lidar_lidar_calib
source install/setup.bash
ros2 launch lidar_lidar_calib lidar_lidar_calib.yaml
```

## Tips for Good Results

- Start with `max_correspondence_distance` large (0.5–1.0 m) to capture the broad basin of convergence, then reduce it (0.1–0.2 m) for precision.
- A good initial guess is critical. Measure the physical mount offsets with a tape measure.
- Run multiple times from the same environment — if the result is consistent, convergence is reliable.
- Verify by visualising `calibrated_cloud` overlaid on the center cloud in RViz.

## Dependencies

- small_gicp
- PCL
- message_filters (for time-synchronized cloud pairs)
