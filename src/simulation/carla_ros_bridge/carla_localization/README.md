# carla_localization

Ground truth localization from CARLA simulation.

## Nodes

### localization

Publishes the ego vehicle's ground truth pose as TF transforms. The transform chain is `map` -> `odom` -> `base_link`, where the `map` -> `odom` transform is identity (no drift in simulation) and `odom` -> `base_link` contains the vehicle's actual pose from CARLA.

Coordinates are converted from CARLA's left-handed system (X-forward, Y-right, Z-up) to ROS's right-handed system (X-forward, Y-left, Z-up).

```bash
ros2 run carla_localization localization
```

**Publications:** `/tf` (`tf2_msgs/TFMessage`)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `carla_host` | string | `localhost` | CARLA server hostname |
| `carla_port` | int | `2000` | CARLA server port |
| `carla_timeout` | double | `10.0` | Connection timeout in seconds |
| `role_name` | string | `ego_vehicle` | Role name of the ego vehicle to track |
| `map_frame` | string | `map` | Name of the map frame |
| `odom_frame` | string | `odom` | Name of the odom frame |
| `base_frame` | string | `base_link` | Name of the base_link frame |
| `publish_rate` | double | `50.0` | TF publish rate in Hz |
