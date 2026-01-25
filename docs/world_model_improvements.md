# World Model Improvements

## Overview

This document describes the improvements made to the `world_model` package to make it route-aware and replace corridor functions with a simpler RouteAhead publisher that enables BFS for downstream lattice planner.

## Changes Summary

| Component | Action |
|-----------|--------|
| Corridor | DELETE - Remove all corridor-related code |
| GetShortestRoute | RENAME to GetShortestRoute (returns entire shortest path) |
| RouteAhead | CREATE - New topic publishing route lanelets ahead |
| LaneContext | MODIFY - Route-aware + heading-aligned fallback |
| RouteAhead Visualization | CREATE - RViz markers for route visualization |

---

## Phase 1: Remove Corridor Functions

### Deleted Files
- `src/world_modeling/lanelet_msgs/msg/Corridor.msg`
- `src/world_modeling/lanelet_msgs/msg/CorridorLane.msg`
- `src/world_modeling/lanelet_msgs/srv/GetCorridor.srv`
- `src/world_modeling/world_model/include/world_model/interfaces/services/corridor_service.hpp`

### Modified Files
- `lanelet_msgs/CMakeLists.txt` - Removed corridor message/service references
- `world_model/lanelet_handler.hpp` - Removed corridor includes and declarations
- `world_model/lanelet_handler.cpp` - Removed implementations:
  - `getCorridor()`
  - `toCorridorMsg()`
  - `buildCorridorLane()`
  - `getBoundaryType()` (corridor-specific version)
- `world_model/world_model_node.cpp` - Removed CorridorService instantiation

---

## Phase 2: Rename GetShortestRoute to GetShortestRoute

### New Files

**`lanelet_msgs/srv/GetShortestRoute.srv`**

```
# Returns the ENTIRE shortest route from ego to goal.
# Requires SetRoute to be called first to establish the destination.
# Returns ALL lanelets from ego's current position to the goal.

---
uint8 TRANSITION_SUCCESSOR=0
uint8 TRANSITION_LEFT=1
uint8 TRANSITION_RIGHT=2

bool success
string error_message
float64 total_length_m
Lanelet[] lanelets
uint8[] transitions
```

**`world_model/interfaces/services/shortest_route_service.hpp`**
- Class `ShortestRouteService`
- Service name: `get_shortest_route`
- No distance parameter - returns ALL lanelets to goal

### Deleted Files
- `lanelet_msgs/srv/GetShortestRoute.srv`
- `world_model/interfaces/services/route_service.hpp`

### Modified Files
- `lanelet_msgs/CMakeLists.txt` - Replaced GetShortestRoute.srv with GetShortestRoute.srv
- `world_model/lanelet_handler.hpp` - Renamed `GetShortestRouteFromPosition()` to `getShortestRoute()`
- `world_model/lanelet_handler.cpp` - Updated implementation to return ALL lanelets
- `world_model/world_model_node.cpp` - Uses `ShortestRouteService`

---

## Phase 3: Create RouteAhead Publisher

### New Files

**`lanelet_msgs/msg/RouteAhead.msg`**

```
# Lanelets ahead of ego that are part of the active route.
# Enables BFS via Lanelet.successor_ids for geometric map building.

std_msgs/Header header
int64[] ids
Lanelet[] lanelets
float64 distance_to_first_m
float64 total_distance_m
bool has_active_route
```

**`world_model/interfaces/publishers/route_ahead_publisher.hpp`**
- Publishes at configurable rate (default 10Hz)
- Uses configurable lookahead distance (default 100m)
- Calls `lanelet_->getRouteAhead(ego_point, lookahead_distance_m)`

### Modified Files
- `lanelet_msgs/CMakeLists.txt` - Added RouteAhead.msg
- `world_model/lanelet_handler.hpp` - Added `GetShortestRouteAhead()` declaration
- `world_model/lanelet_handler.cpp` - Implemented `GetShortestRouteAhead()`
- `world_model/world_model_node.cpp` - Added parameters and RouteAheadPublisher

### New Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `route_ahead_publish_rate_hz` | 10.0 | Publication rate |
| `route_ahead_lookahead_m` | 100.0 | Lookahead distance for route lanelets |

---

## Phase 4: Make LaneContext Route-Aware

### New Method: `findCurrentLaneletId()`

```cpp
std::optional<int64_t> findCurrentLaneletId(
  const geometry_msgs::msg::Point & point,
  double heading_rad,
  double route_priority_threshold_m = 10.0,
  double heading_search_radius_m = 15.0) const;
```

**Logic:**
1. **If route exists**: Find lanelet from `active_route_` that ego is on/near (within threshold)
2. **If no route (fallback)**: Pick lanelet whose centerline tangent best aligns with vehicle heading
3. **Ultimate fallback**: Return nearest lanelet

### Modified Files
- `world_model/lanelet_handler.hpp` - Added declaration
- `world_model/lanelet_handler.cpp` - Implemented route-priority + heading-aligned selection
- `world_model/lane_context_publisher.hpp` - Uses `findCurrentLaneletId()` with ego yaw extracted from quaternion

### Benefits
- When on a route, LaneContext reports the route lanelet even if a parallel lane is closer
- Without a route, picks the lanelet aligned with vehicle heading (useful at intersections)

---

## Phase 5: RouteAhead Visualization

### New Files
- `src/infrastructure/lanelet_markers/include/lanelet_markers/route_ahead_markers_node.hpp`
- `src/infrastructure/lanelet_markers/src/route_ahead_markers_node.cpp`

### Modified Files
- `lanelet_markers/CMakeLists.txt` - Added `route_ahead_markers_node` executable
- `lanelet_markers/launch/lanelet_viz_markers.launch.yaml` - Added node to launch
- `lanelet_markers/config/lanelet_viz_markers.yaml` - Added node parameters

### Visualization Features
- **Route path overlay** - Cyan highlighted centerline (0.4m wide)
- **Route info text** - Shows "Route: X lanelets | Ym" at route start
- **Lanelet IDs** - Optional, disabled by default

### Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/route_ahead` (subscribe) | `lanelet_msgs/msg/RouteAhead` | Route lanelets from world_model |
| `/route_ahead_markers` (publish) | `visualization_msgs/msg/MarkerArray` | RViz markers |

### Parameters
| Parameter | Default | Description |
|-----------|---------|-------------|
| `frame_id` | "map" | Coordinate frame |
| `centerline_line_width` | 0.4 | Width of route path overlay |
| `show_lanelet_ids` | false | Show lanelet ID labels |
| `show_route_info` | true | Show route summary text |

---

## Architecture

### Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/route_ahead` | RouteAhead | world_model | Lanelets within lookahead distance |
| `/lane_context` | CurrentLaneContext | world_model | Current lanelet (route-aware) |
| `/route_ahead_markers` | MarkerArray | lanelet_markers | RViz visualization |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/set_route` | SetRoute | Sets goal point, computes and caches route |
| `/get_shortest_route` | GetShortestRoute | Returns entire cached route from ego to goal |
| `/get_lanelets_by_reg_elem` | GetLaneletsByRegElem | Get lanelets by regulatory element ID |

### Workflow

1. Call `/set_route` with goal point → caches shortest path
2. `/route_ahead` topic publishes lanelets within lookahead distance
3. `/lane_context` reports current lanelet (prioritizes route lanelets)
4. Call `/get_shortest_route` for full route to goal
5. Visualize with `/route_ahead_markers` in RViz

---

## Building

```bash
# In devcontainer
colcon build --packages-select lanelet_msgs world_model lanelet_markers
```

## Testing

```bash
# Set a route
ros2 service call /set_route lanelet_msgs/srv/SetRoute "{goal_point: {x: 100, y: 200, z: 0}}"

# Check RouteAhead publisher
ros2 topic echo /route_ahead

# Get full route
ros2 service call /get_shortest_route lanelet_msgs/srv/GetShortestRoute

# Visualize in RViz
ros2 launch lanelet_markers lanelet_viz_markers.launch.yaml
# Add MarkerArray display for /route_ahead_markers
```

## Design Decisions

### Why remove Corridor?
- Corridor was complex and tightly coupled
- RouteAhead + Lanelet.successor_ids enables simpler BFS in lattice planner
- Downstream consumers can build their own geometric representation

### Why multiple successor_ids?
- A lanelet can have multiple successors at divergence points (intersections, exits, lane splits)
- The BFS in lattice planner filters successors based on which one is on the route
- Keeping full graph connectivity allows exploration of alternatives

### Why heading-aligned fallback?
- Without a route, nearest lanelet may be wrong at intersections or parallel roads
- Heading alignment picks the lanelet the vehicle is actually driving on
- Combined with distance weighting for robustness
