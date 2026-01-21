# World Model

ROS2 Lifecycle Node that maintains a unified view of the world for autonomous driving.

## Overview

The world model aggregates data from multiple sources (perception, prediction, HD map) into a single queryable state. It provides:

- **Entity tracking**: Cars, humans, bicycles, motorcycles, traffic lights
- **Lanelet map queries**: Routes, corridors, regulatory elements
- **Lane context**: Current lanelet, distances to events

## Quick Start

```bash
# Build
colcon build --packages-select world_model

# Run
ros2 run world_model world_model_node --ros-args \
  -p osm_map_path:=/path/to/map.osm \
  -p map_frame:=map \
  -p base_frame:=base_link

# Lifecycle management
ros2 lifecycle set /world_model configure
ros2 lifecycle set /world_model activate
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `osm_map_path` | string | "" | Path to OSM lanelet map |
| `map_frame` | string | "map" | Map frame ID |
| `base_frame` | string | "base_link" | Robot base frame ID |
| `utm_frame` | string | "utm" | UTM frame for map loading |
| `entity_history_duration_sec` | double | 5.0 | How long to keep entity history |
| `entity_prune_timeout_sec` | double | 2.0 | Prune entities not seen for this long |
| `traffic_light_timeout_sec` | double | 1.0 | Traffic light prune timeout |
| `cleanup_interval_ms` | double | 1000.0 | Cleanup worker interval |
| `lane_context_publish_rate_hz` | double | 10.0 | Lane context publish rate |
| `map_viz_publish_rate_hz` | double | 1.0 | Map visualization publish rate |
| `map_viz_radius_m` | double | 100.0 | Radius for map visualization |

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
| `lane_context` | `lanelet_msgs/CurrentLaneContext` | Current lane information |
| `map_visualization` | `lanelet_msgs/MapVisualization` | Nearby lanelets for viz |

## Services

| Service | Type | Description |
|---------|------|-------------|
| `get_route` | `lanelet_msgs/GetRoute` | Compute route between lanelets |
| `get_corridor` | `lanelet_msgs/GetCorridor` | Get driving corridor |
| `get_lanelets_by_reg_elem` | `lanelet_msgs/GetLaneletsByRegElem` | Find lanelets by regulatory element |

## Architecture

See [ARCHITECTURE.md](ARCHITECTURE.md) for detailed design documentation.

## License

Apache 2.0 - Copyright (c) 2025-present WATonomous
