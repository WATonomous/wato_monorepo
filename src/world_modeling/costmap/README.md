# Costmap

ROS2 Lifecycle Node that composes a 2D occupancy grid from pluggable layers for local obstacle avoidance.

## Overview

The costmap node builds a `nav_msgs/OccupancyGrid` centred on the vehicle at a configurable rate. Each registered layer (objects, pointcloud, virtual walls) writes obstacle costs into the shared grid. The node also publishes the vehicle footprint as a `geometry_msgs/PolygonStamped`.

## Parameters

### Node-level

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `costmap_frame` | string | `"base_link"` | Frame the grid is centred on |
| `map_frame` | string | `"map"` | Map frame for TF lookups |
| `publish_rate_hz` | double | 20.0 | Grid publish frequency |
| `grid_width_m` | double | 60.0 | Grid width in metres |
| `grid_height_m` | double | 60.0 | Grid height in metres |
| `resolution` | double | 0.25 | Cell size in metres |
| `footprint_front_left` | double[] | [0.0, 0.0] | Front-left corner [x, y] in costmap_frame |
| `footprint_rear_right` | double[] | [0.0, 0.0] | Rear-right corner [x, y] in costmap_frame |
| `layers` | string[] | ["objects", "virtual_wall"] | Active layer names |

### ObjectsLayer (`layers.<name>.`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `bbox_inflation_m` | double | 0.5 | Inflate bounding boxes by this amount |
| `bbox_cost_decay` | double | 1.0 | Cost multiplier for inflated region |
| `prediction_inflation_m` | double | 0.3 | Radius around predicted waypoints |
| `prediction_cost_decay` | double | 0.8 | Per-step cost decay along predictions |

### PointCloudLayer (`layers.<name>.`)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `max_height_m` | double | 0.0 | Ignore points above this height (0 = disabled) |
| `min_height_m` | double | 0.0 | Ignore points below this height (0 = disabled) |
| `inflation_m` | double | 0.0 | Inflate each point by this radius |
| `cost_decay` | double | 0.0 | Cost decay rate for inflated cells |

The pointcloud layer also reads the node-level `footprint_front_left` / `footprint_rear_right` params and excludes points inside the vehicle footprint rectangle.

### VirtualWallLayer

No parameters. Walls are created and removed via ROS services.

## Topics

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `costmap` | `nav_msgs/OccupancyGrid` | Composed occupancy grid |
| `footprint` | `geometry_msgs/PolygonStamped` | Vehicle footprint polygon |

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `world_objects` | `world_model_msgs/WorldObjectArray` | Tracked objects (ObjectsLayer) |
| `pointcloud` | `sensor_msgs/PointCloud2` | LiDAR point cloud (PointCloudLayer) |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `spawn_wall` | `costmap_msgs/SpawnWall` | Create a virtual wall |
| `despawn_wall` | `costmap_msgs/DespawnWall` | Remove a virtual wall |
