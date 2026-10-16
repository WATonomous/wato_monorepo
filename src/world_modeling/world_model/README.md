# World Model

ROS2 Lifecycle Node that maintains a unified view of the world for autonomous driving.

## Overview

The world model aggregates data from multiple sources (perception, prediction, HD map) into a single queryable state. It provides:

- **Entity tracking**: Cars, humans, bicycles, motorcycles, traffic lights
- **Lanelet map queries**: Routes, corridors, regulatory elements
- **Lane context**: Current lanelet, distances to events

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `osm_map_path` | string | "" | Path to OSM lanelet map |
| `map_frame` | string | "map" | Map frame ID |
| `base_frame` | string | "base_footprint" | Robot base frame ID |
| `projector_type` | string | "utm" | Projector: "utm" or "local_cartesian" |
| `alignment_mode` | string | "fixed" | "fixed" uses the configured origin directly; "tf" applies a full projected-frame-to-map transform |
| `projected_frame` | string | "utm" | Source TF frame in "tf" mode; absolute UTM for the UTM projector, configured local ENU for Local Cartesian |
| `origin_lat` | double | 0.0 | Geographic origin; in UTM TF mode, selects the UTM zone only |
| `origin_lon` | double | 0.0 | Geographic origin; in UTM TF mode, selects the UTM zone only |
| `origin_alt` | double | 0.0 | Origin altitude in metres; subtracted in fixed mode, not in UTM TF mode |
| `entity_history_duration_sec` | double | 5.0 | How long to keep entity history |
| `entity_prune_timeout_sec` | double | 2.0 | Prune entities not seen for this long |
| `traffic_light_timeout_sec` | double | 1.0 | Traffic light prune timeout |
| `cleanup_interval_ms` | double | 1000.0 | Cleanup worker interval |
| `lane_context_publish_rate_hz` | double | 10.0 | Lane context publish rate |
| `map_viz_publish_rate_hz` | double | 1.0 | Map visualization publish rate |
| `map_viz_radius_m` | double | 100.0 | Radius for map visualization |
| `world_objects_enriched_publish_rate_hz` | double | 10.0 | World objects enriched publish rate |
| `route_ahead_publish_rate_hz` | double | 10.0 | Route ahead publish rate |
| `route_ahead_lookahead_m` | double | 100.0 | Lookahead distance for route lanelets |

## Projection and map alignment

`projector_type` converts OSM latitude/longitude into metric coordinates.
`alignment_mode` specifies how those coordinates relate to the ROS `map_frame`.
Both projectors honour the configured latitude, longitude, and altitude.

With `alignment_mode: fixed`, projection output is already in `map_frame`:
UTM subtracts the projected geographic origin and origin altitude; Local
Cartesian produces ENU coordinates relative to that origin. No alignment TF
is required. CARLA maps must use the same projection and origin as their XODR.

With `alignment_mode: tf`, the node waits for
`lookupTransform(map_frame, projected_frame, ...)` and applies its complete
rotation and translation to every OSM point during parsing. UTM projection
produces **absolute** easting, northing, and OSM elevation, so Eidos's absolute
UTM transform is applied once without subtracting another origin. For Local
Cartesian, the source frame must describe ENU coordinates relative to the
configured origin. All Lanelet2 geometry and indices are constructed after
alignment. The transform is sampled once per map load; deactivate, clean up,
configure, and activate again to load a different map or realign after a TF
change.

Unsupported modes, invalid origins, and invalid transforms fail explicitly.
A missing TF is retried; a bad map file stops the load with an error.

Named oval, Town10HD, and test-track configs and launch examples are documented
in [world_modeling_bringup](../world_modeling_bringup/DEVELOPING.md).
The former `wait_for_utm_transform` and `utm_frame` parameters are replaced by
`alignment_mode` and `projected_frame`. The unused `utm_origin_x/y` arguments
have been removed from the loader API.

## Topics

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `detections` | `vision_msgs/Detection3DArray` | 3D object detections |
| `traffic_light_detections` | `vision_msgs/Detection2DArray` | Traffic light detections |
| `predictions` | `prediction_msgs/PredictionHypothesesArray` | Predicted trajectories |

### Published
| Topic | Type | Description |
|-------|------|-------------|
| `lane_context` | `lanelet_msgs/CurrentLaneContext` | Current lane information (route-aware) |
| `route_ahead` | `lanelet_msgs/RouteAhead` | Route lanelets ahead within lookahead distance |
| `map_visualization` | `lanelet_msgs/MapVisualization` | Nearby lanelets for viz |
| `world_objects_enriched` | `world_model_msgs/WorldObjectArray` | All tracked entities (enriched) |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `set_route` | `lanelet_msgs/SetRoute` | Set destination and cache route |
| `get_shortest_route` | `lanelet_msgs/GetShortestRoute` | Get entire shortest route from ego to goal |
| `get_lanelets_by_reg_elem` | `lanelet_msgs/GetLaneletsByRegElem` | Find lanelets by regulatory element |

## License

Apache 2.0 - Copyright (c) 2025-present WATonomous
