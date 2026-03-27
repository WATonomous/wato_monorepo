# KeyframeMapVisualization

**Class:** `eidos::KeyframeMapVisualization`
**XML:** `visualization_plugins.xml`

Publishes a composite point cloud map by transforming stored keyframe body-frame clouds into world frame using current optimized poses. Supports two modes: `"accumulate"` builds up the map over time (with a configurable skip factor to reduce density), and `"windowed"` only publishes clouds within a spatial radius of the latest keyframe.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topic` | string | `"slam/visualization/map"` | Published PointCloud2 topic. |
| `pointcloud_from` | string | `"liso_factor"` | MapManager data key prefix for retrieving keyframe point clouds. |
| `voxel_leaf_size` | double | `0.4` | Voxel downsample leaf size (meters). |
| `publish_rate` | double | `1.0` | Publish rate (Hz). |
| `mode` | string | `"accumulate"` | Visualization mode: `"accumulate"` or `"windowed"`. |
| `accumulate.skip_factor` | int | `5` | In accumulate mode, include every Nth keyframe to reduce density. |
| `windowed.radius` | double | `50.0` | In windowed mode, spatial radius (meters) around the latest keyframe. |

## Notes

- In accumulate mode, the full map is rebuilt from body-frame clouds plus current poses on every render, so graph corrections (GPS, loop closure) are immediately reflected.
- In windowed mode, the latest keyframe must exist in the optimized values.

## Mapping vs Localization

Loaded in both modes. In **mapping** mode, it visualizes the growing map using optimized keyframe poses from ISAM2. In **localization** mode, factor plugins typically have `add_factors: false`, so no new keyframes are added to the graph and no ISAM2 optimization runs. The visualization uses the prior map's stored keyframe poses to render the loaded map. If the prior map has keyframes with stored point clouds, the full map is visible from activation.
