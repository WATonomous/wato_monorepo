# World Model Architecture

## Design Principles

1. **Separation of concerns**: Storage (WorldState), queries (LaneletHandler), I/O (Interfaces)
2. **Thread safety**: Concurrent access managed at the data layer
3. **Reactive processing**: Data enrichment happens on arrival, not periodically
4. **Lifecycle management**: All components follow ROS2 lifecycle patterns

## Component Overview

```
┌─────────────────────────────────────────────────────────────────────────┐
│                          WorldModelNode                                  │
│                     (Lifecycle Orchestrator)                             │
├─────────────────────────────────────────────────────────────────────────┤
│                                                                          │
│  ┌──────────────┐    ┌─────────────────┐    ┌────────────────────────┐  │
│  │  WorldState  │    │ LaneletHandler  │    │   TF Buffer            │  │
│  │  (Storage)   │    │ (Map Queries)   │    │   (Transforms)         │  │
│  └──────┬───────┘    └────────┬────────┘    └───────────┬────────────┘  │
│         │                     │                         │               │
│         ▼                     ▼                         ▼               │
│  ┌──────────────────────────────────────────────────────────────────┐   │
│  │                        Interfaces                                 │   │
│  │  ┌────────────┐  ┌────────────┐  ┌──────────┐  ┌───────────┐    │   │
│  │  │ Subscribers│  │ Publishers │  │ Services │  │  Workers  │    │   │
│  │  └────────────┘  └────────────┘  └──────────┘  └───────────┘    │   │
│  └──────────────────────────────────────────────────────────────────┘   │
│                                                                          │
└─────────────────────────────────────────────────────────────────────────┘
```

## Core Components

### WorldState (`world_state.hpp`)

Pure data storage with generic buffer access. Contains `EntityBuffer<T>` for each entity type.

```cpp
WorldState state;
auto& cars = state.buffer<Car>();      // Get car buffer
auto& humans = state.buffer<Human>();  // Get human buffer
```

### EntityBuffer (`types/entity_buffer.hpp`)

Thread-safe container using `std::shared_mutex` for reader-writer locking.

**Key operations:**
- `upsert(id, default, modifier)` - Atomic insert-or-update (used by subscribers)
- `modify(id, modifier)` - Atomic update-if-exists (used by prediction subscriber)
- `prune(predicate)` - Remove entities matching predicate (used by cleanup worker)
- `get(id)` / `getAll()` - Read operations

**Locking behavior:**
- Read operations: `shared_lock` (multiple readers allowed)
- Write operations: `unique_lock` (exclusive access)

### LaneletHandler (`lanelet_handler.hpp`)

Wrapper around Lanelet2 for map queries. **Immutable after `loadMap()`** - safe for concurrent reads.

**Capabilities:**
- Spatial queries: `findNearestLanelet()`, `getLaneletsInRadius()`
- Routing: `getRoute()`, `getCorridor()`
- Regulatory elements: `getLaneletsByRegElem()`
- Conversions: `toLaneletMsg()`

### InterfaceBase (`interfaces/interface_base.hpp`)

Base class for all interface components. Provides:

1. **Lifecycle methods**: `activate()` / `deactivate()`
2. **Accessor wrappers** for type-safe access:
   - `WorldStateReader` - const access to buffers
   - `WorldStateWriter` - mutable access to buffers
   - `LaneletReader` - access to lanelet queries

## Entity Type Hierarchy

```
Entity (base)
├── Entity3D (3D tracked objects)
│   ├── Car
│   ├── Human
│   ├── Bicycle
│   └── Motorcycle
└── Entity2D (2D image detections)
    └── TrafficLight
```

**Common fields:**
- `history` - Detection history (deque, front = most recent)
- `lanelet_id` - Associated lanelet (set on detection arrival)

**Entity3D specific:**
- `predictions` - Predicted future trajectories

**TrafficLight specific:**
- `state` - Interpreted state (RED/YELLOW/GREEN/UNKNOWN)
- `confidence` - Classification confidence
- `reg_elem_id` - Lanelet regulatory element ID

## Data Flow

### Detection Processing

```
Detection3DArray arrives
        │
        ▼
DetectionSubscriber.onMessage()
        │
        ▼
classify() → determine entity type (Car/Human/Bicycle/Motorcycle)
        │
        ▼
buffer.upsert() [ATOMIC - holds unique_lock]
    ├── Push detection to history
    ├── Trim old history (by duration)
    └── Enrich: set lanelet_id via findNearestLaneletId()
```

### Prediction Processing

```
PredictionHypothesesArray arrives
        │
        ▼
PredictionSubscriber.onMessage()
        │
        ▼
For each prediction:
    buffer<Car>.modify() || buffer<Human>.modify() || ...
        │
        ▼
    Set entity.predictions = preds [ATOMIC]
```

### Cleanup Processing

```
CleanupWorker runs every cleanup_interval_ms
        │
        ▼
For each entity type:
    buffer.prune(should_remove) [ATOMIC]
        │
        ▼
    Remove entities not seen for > timeout
```

## Concurrency Model

### Thread Types

| Thread | Components | Access Pattern |
|--------|------------|----------------|
| ROS Executor | Subscribers, Publishers, Services | MultiThreadedExecutor |
| Dedicated | CleanupWorker | Own thread with sleep interval |

### Safety Guarantees

1. **EntityBuffer operations are atomic** - Each method acquires its own lock
2. **No read-modify-write races** - Subscribers use `upsert()`/`modify()` with lambdas
3. **LaneletHandler is immutable** - Safe for concurrent reads after `loadMap()`

### Access Patterns by Component

| Component | WorldState | LaneletHandler |
|-----------|------------|----------------|
| DetectionSubscriber | Write (upsert) | Read (findNearestLaneletId) |
| TrafficLightSubscriber | Write (upsert) | - |
| PredictionSubscriber | Write (modify) | - |
| CleanupWorker | Write (prune) | - |
| LaneContextPublisher | - | Read |
| MapVizPublisher | - | Read |
| Services | - | Read |

## File Structure

```
world_model/
├── include/world_model/
│   ├── world_model_node.hpp      # Lifecycle node
│   ├── world_state.hpp           # Entity storage
│   ├── lanelet_handler.hpp       # Map queries
│   ├── types/
│   │   ├── types.hpp             # Aggregate header
│   │   ├── entity.hpp            # Base entity
│   │   ├── entity_2d.hpp         # 2D entities (TrafficLight)
│   │   ├── entity_3d.hpp         # 3D entities (Car, Human, etc.)
│   │   ├── entity_buffer.hpp     # Thread-safe container
│   │   └── entity_type.hpp       # EntityType enum
│   └── interfaces/
│       ├── interface_base.hpp    # Base + accessor wrappers
│       ├── publishers/
│       │   ├── lane_context_publisher.hpp
│       │   └── map_viz_publisher.hpp
│       ├── subscribers/
│       │   ├── detection_subscriber.hpp
│       │   ├── traffic_light_subscriber.hpp
│       │   └── prediction_subscriber.hpp
│       ├── services/
│       │   ├── route_service.hpp
│       │   ├── corridor_service.hpp
│       │   └── reg_elem_service.hpp
│       └── workers/
│           └── cleanup_worker.hpp
└── src/
    ├── world_model_node.cpp      # Node implementation
    └── lanelet_handler.cpp       # LaneletHandler implementation
```

## Executor Model

The node uses a `MultiThreadedExecutor` (not a component) to enable:
- Callback groups to provide mutual exclusion
- Concurrent handling of different callback types
- CleanupWorker running on its own thread

```cpp
// main() in world_model_node.cpp
rclcpp::executors::MultiThreadedExecutor executor;
executor.add_node(node->get_node_base_interface());
executor.spin();
```

## Future Work / TODOs

1. **Lifecycle compliance for Subscribers/Services**: Currently, subscriptions and services are created in constructors (during `configure`), meaning they start processing before `activate()`. To be fully lifecycle-compliant, these should be created in `activate()` and destroyed in `deactivate()`.

2. **LaneContextPublisher**: Calculate actual `arc_length`, `lateral_offset`, `heading_error` (currently hardcoded to 0)

3. **Distance to events**: Calculate actual distances to traffic lights, stop lines, intersections along route (currently 0 if on current lanelet, -1 otherwise)

4. **Traffic light association**: Link detected traffic lights to lanelet regulatory elements via `reg_elem_id`

5. **Entity queries**: Add spatial queries to WorldState (e.g., get entities in radius)

6. **Performance**: If lanelet lookup in detection callback becomes a bottleneck, consider:
   - Caching recent lookups
   - Spatial indexing for entities
