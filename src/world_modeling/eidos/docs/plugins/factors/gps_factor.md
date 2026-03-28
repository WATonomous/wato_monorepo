# GpsFactor

**Class:** `eidos::GpsFactor`
**Type:** Factor (latching via `latchFactor`)
**XML:** `factor_plugins.xml`

Subscribes to NavSatFix and IMU. Converts GPS to UTM, applies a yaw-compensated rotation and offset to produce map-frame coordinates, then attaches unary `gtsam::GPSFactor` entries to states created by other plugins. On first accepted fix, the UTM-to-map offset is computed from the current estimator pose and the IMU heading, then persisted via MapManager. Registers `gps_factor/position`, `gps_factor/utm_position`, and global `gps_factor/utm_to_map` for persistence. Publishes the utm-to-map transform as a `geometry_msgs/TransformStamped` on the `gps_factor/utm_to_map_tf` topic (transient-local QoS) for `eidos_transform` to consume and broadcast as a static TF. This plugin does not use a `StaticTransformBroadcaster`.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `gps_topic` | string | `"gps/fix"` | Input NavSatFix topic. |
| `imu_topic` | string | `"imu/data"` | IMU topic for heading (yaw) used to initialize the UTM-to-map rotation. |
| `add_factors` | bool | `true` | Whether to add GPS factors to the graph. When false, GPS data is still tracked and persisted but no factors are created. |
| `max_cov` | double | `2.0` | Maximum accepted position covariance from the GPS sensor. Fixes with higher covariance are rejected. |
| `use_elevation` | bool | `false` | Whether to use GPS elevation in the factor. When false, z noise is set very high so the factor has no z influence. |
| `min_radius` | double | `5.0` | Minimum distance (meters) between consecutive GPS factors (distance gate). |
| `gps_cov` | double[3] | `[1.0, 1.0, 1.0]` | Minimum noise floor variances [x, y, z] for the GPS factor. Actual noise is `max(sensor_cov, gps_cov)`. |
| `pose_cov_threshold` | double | `25.0` | Pose covariance threshold. |

## Notes

- GPS never creates its own states in the graph; it only latches onto states created by other plugins (e.g. LISO).
- The UTM-to-map offset is persisted as a global entry in MapManager and is reloaded on `activate()` when a prior map is loaded.
- The `utm -> map` transform is published on the `gps_factor/utm_to_map_tf` topic as a `TransformStamped` (not via a `StaticTransformBroadcaster`). `eidos_transform` subscribes to this topic and broadcasts it as a static TF.
- A time window of 0.2 seconds is used to match GPS fixes to state timestamps.
- The GPS fix queue is capped at 100 messages. When a GPS factor is accepted, `result.correction = true` is set, signaling the optimizer to run extra correction iterations.

## Mapping vs Localization

| Parameter | Mapping | Localization |
|---|---|---|
| `add_factors` | `true` | `false` |
| `min_radius` | `5.0` | `200.0` |

In **mapping**, GPS adds unary position constraints to the graph at regular intervals (every `min_radius` meters of travel). These constraints prevent drift over long distances and anchor the map to a global coordinate frame.

In **localization**, GPS does not add factors. It still loads the saved `utm_to_map` offset from the prior map so the data is available for `eidos_transform` to broadcast the `utm -> map` static TF (needed by downstream consumers such as the world model for lanelet map loading). The `min_radius` is set high since no factors are produced anyway.
