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

## After Launching

1. **Ensure the vehicle is stationary** and the environment has geometric features (walls, objects — not an empty room).

2. **Watch the node log** for transform output. The calibrated transform is printed each time GICP converges on a new scan pair.

3. **Verify alignment visually in Foxglove** — open a 3D panel and add both `calibrated_cloud` and the center LiDAR topic. The two clouds should visually overlap with no offset.

4. **Record multiple runs** — restart the node 5–10 times from the same position and compare the reported transforms.

## Definition of Good Result

**Visual check (Foxglove):** The `calibrated_cloud` overlays cleanly on the center cloud — walls, edges, and objects from both clouds land on the same surfaces with no ghosting.

**Quantitative convergence** — run the node 5–10 times and compare the reported transforms:

| Quantity | Good | Poor |
|----------|------|------|
| Translation std dev across runs | < 0.01 m | > 0.03 m |
| Rotation std dev across runs | < 0.3 deg | > 1.0 deg |

If results are inconsistent across runs:
- Increase `max_correspondence_distance` — the initial guess may be outside the convergence basin
- Check that the environment has enough geometric features (flat open areas give poor constraints)
- Verify the vehicle was stationary for all runs (any motion during capture corrupts the alignment)

Once stable, update the `<origin>` in `eve_description/urdf/` with the mean transform and set `max_correspondence_distance` to 0.1–0.2 m for a final precision pass.

## Dependencies

- small_gicp
- PCL
- message_filters (for time-synchronized cloud pairs)
