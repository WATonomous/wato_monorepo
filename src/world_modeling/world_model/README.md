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
| `base_frame` | string | "base_link" | Robot base frame ID |
| `utm_frame` | string | "utm" | UTM frame for map loading |
| `projector_type` | string | "utm" | Projector: "utm" or "local_cartesian" |
| `entity_history_duration_sec` | double | 5.0 | How long to keep entity history |
| `entity_prune_timeout_sec` | double | 2.0 | Prune entities not seen for this long |
| `traffic_light_timeout_sec` | double | 1.0 | Traffic light prune timeout |
| `cleanup_interval_ms` | double | 1000.0 | Cleanup worker interval |
| `lane_context_publish_rate_hz` | double | 10.0 | Lane context publish rate |
| `map_viz_publish_rate_hz` | double | 1.0 | Map visualization publish rate |
| `map_viz_radius_m` | double | 100.0 | Radius for map visualization |
| `dynamic_objects_publish_rate_hz` | double | 10.0 | Dynamic objects publish rate |
| `route_ahead_publish_rate_hz` | double | 10.0 | Route ahead publish rate |
| `route_ahead_lookahead_m` | double | 100.0 | Lookahead distance for route lanelets |

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
| `dynamic_objects` | `world_model_msgs/DynamicObjectArray` | All tracked entities |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `set_route` | `lanelet_msgs/SetRoute` | Set destination and cache route |
| `get_shortest_route` | `lanelet_msgs/GetShortestRoute` | Get entire shortest route from ego to goal |
| `get_lanelets_by_reg_elem` | `lanelet_msgs/GetLaneletsByRegElem` | Find lanelets by regulatory element |

## License

Apache 2.0 - Copyright (c) 2025-present WATonomous
