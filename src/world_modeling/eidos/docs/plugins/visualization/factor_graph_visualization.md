# FactorGraphVisualization

**Class:** `eidos::FactorGraphVisualization`
**XML:** `visualization_plugins.xml`

Publishes a `MarkerArray` visualizing the factor graph in RViz. Renders state nodes as colored spheres (colored by owner plugin), GPS-constrained states as larger magenta spheres, text labels with state indices, and edges from the adjacency graph as colored lines (colored by edge owner plugin). Sends a DELETEALL marker before each publish to clear stale markers. Only publishes when at least one subscriber is connected.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `topic` | string | `"slam/visualization/factor_graph"` | Published MarkerArray topic. |
| `state_scale` | double | `1.0` | Sphere diameter for state markers. |
| `line_width` | double | `0.5` | Line width for edge markers. |
| `publish_rate` | double | `1.0` | Publish rate (Hz). |
| `mode` | string | `"full"` | Visualization mode (currently only `"full"` is implemented). |
| `window_radius` | double | `50.0` | Spatial radius for windowed mode (declared, not yet used in `"full"` mode). |

## Mapping vs Localization

Only loaded in mapping mode. In localization mode, no new factors are added to the graph, so factor graph visualization is not useful and is omitted from the plugin list.
