# World Model - Developer Guide

## Overview

The world model is a ROS 2 **Lifecycle Node** that maintains a unified, thread-safe view of the world for autonomous driving. It aggregates data from perception, prediction, and HD map sources into queryable entities exposed through publishers and services.

```
                             ┌──────────────────────────────────────────┐
                             │            WorldModelNode                │
                             │         (Lifecycle orchestrator)         │
                             ├────────────┬─────────────┬──────────────┤
  world_object_seeds ───────►│  Writer    │ WorldState  │ LaneletHandler│
  (WorldObjectArray)         │ (pipeline) │ (entities)  │ (map queries) │
                             ├────────────┴──────┬──────┴──────────────┤
                             │              Interfaces                  │
                             │   ┌──────────────┐ ┌──────────────────┐ │
                             │   │  Publishers   │ │    Services      │ │
                             │   │ (timer-based) │ │ (request/reply)  │ │
                             │   └──────┬───────┘ └────────┬─────────┘ │
                             └──────────┼──────────────────┼───────────┘
                                        ▼                  ▼
                              Topics (periodic)    Services (on-demand)
```

## Core Concepts

### Entity Types

All tracked objects inherit from `Entity3D`, which stores a detection history (deque, newest-first), predicted paths, and optional lanelet context:

| Type | Enrichment | Notes |
|------|-----------|-------|
| `Unknown` | None | Unclassified detections |
| `Car` | Tracked lane finding | Includes vehicles and trucks |
| `Human` | None | Pedestrians |
| `Bicycle` | Tracked lane finding | Cyclists |
| `Motorcycle` | Tracked lane finding | |
| `TrafficLight` | Map matching | Has state (RED/YELLOW/GREEN), way_id, reg_elem_id |

Classification is string-based from `detection.results[hypothesis_idx].hypothesis.class_id`:
- `"car"`, `"vehicle"`, `"truck"` -> CAR
- `"person"`, `"pedestrian"`, `"human"` -> HUMAN
- `"bicycle"`, `"cyclist"` -> BICYCLE
- `"motorcycle"`, `"motorbike"` -> MOTORCYCLE
- `"traffic_light"` -> TRAFFIC_LIGHT

### WorldState and Copy-on-Write Concurrency

`WorldState` owns one `EntityBuffer<T>` per entity type. The buffer uses a **copy-on-write** pattern with `std::atomic_load`/`std::atomic_store` on a `shared_ptr<const Map>`:

```
Readers (publishers, services):              Writers (inbound pipeline):
  snapshot = atomic_load(data_)                lock(write_mutex_)
  iterate snapshot (no lock needed)            copy = make_shared(*atomic_load(data_))
                                               mutate(copy)
                                               atomic_store(data_, copy)
                                               unlock
```

This means publishers and services get lock-free reads of a consistent snapshot, while the single writer holds a mutex only during mutation. The `batch()` method is critical: it copies the map once for many mutations, avoiding O(n) copies when processing an entire detection message.

Access is mediated through wrapper classes:
- `WorldStateReader` (const access) - used by publishers and services
- `WorldStateWriter` (mutable access) - used by the inbound writer only

### Lanelet Handler

`LaneletHandler` wraps Lanelet2 for HD map queries. It supports two projector types:
- **`utm`** (production): UTM projector with GPS-derived origin offset from TF
- **`local_cartesian`** (CARLA/simulation): local Cartesian with zero offset

Key query capabilities:
- **Lane finding**: `findCurrentLaneletId()` uses route priority, BFS from previous lanelet, and heading alignment for stable frame-to-frame tracking
- **Radius queries**: `getLaneletsInRadius()`, `getReachableLaneletsInRadius()` (BFS via routing graph)
- **Traffic light matching**: `findNearestTrafficLightMatch()` returns way_id + reg_elem_id
- **Routing**: `setActiveRoute()`, `getShortestRoute()`, `getRouteAhead()`

The map loads asynchronously after activation via a wall timer that waits for the utm-to-map TF transform.

## Interface System

All publishers, services, and the subscriber implement `InterfaceBase`:

```cpp
class InterfaceBase {
  virtual void activate() {}    // Start timers, enable pubs
  virtual void deactivate() {}  // Cancel timers
};
```

The node stores them as `std::vector<std::unique_ptr<InterfaceBase>>` and calls `activate()`/`deactivate()` during lifecycle transitions.

## Inbound Pipeline

There is a **single inbound subscription** on the `world_object_seeds` topic (`WorldObjectArray`). The `WorldModelWriter` classifies each object by type, then runs a three-stage pipeline per type inside a single COW `batch()` call:

```
WorldObjectArray message
        │
        ▼
   ┌─────────┐
   │ Classify │  Map class_id string to EntityType
   └────┬────┘
        │  (per entity type)
        ▼
   ┌──────────────────────────────────────┐
   │          batch() — single COW copy   │
   │  ┌──────────┐                        │
   │  │ Populate  │  Upsert detections,   │
   │  │          │  store predictions      │
   │  └────┬─────┘                        │
   │       ▼                              │
   │  ┌──────────┐                        │
   │  │ Enrich   │  Assign lanelet_id,    │
   │  │          │  match traffic lights   │
   │  └────┬─────┘                        │
   │       ▼                              │
   │  ┌───────────┐                       │
   │  │Permanence │  Trim old history,    │
   │  │           │  prune stale entities  │
   │  └───────────┘                       │
   └──────────────────────────────────────┘
```

## Threading Model

The node uses `MultiThreadedExecutor` with callback groups:
- **WorldModelWriter**: `Reentrant` callback group (non-blocking to timer threads)
- **Publishers**: Wall timers (default `MutuallyExclusive` unless overridden)
- **Services**: Default callback group

The COW pattern ensures publishers never block on the writer, and the writer never blocks on publishers.

## Adding a New Interface

### New Publisher

1. Create `include/world_model/interfaces/publishers/my_publisher.hpp`
2. Inherit from `InterfaceBase`, override `activate()` and `deactivate()`
3. Accept `WorldState*` and/or `LaneletHandler*` in the constructor, wrap with `WorldStateReader`
4. Create a wall timer in the constructor (store it); start/cancel in `activate()`/`deactivate()`
5. Register in `WorldModelNode::createInterfaces()` (`world_model_node.cpp`)

```cpp
class MyPublisher : public InterfaceBase {
public:
  MyPublisher(rclcpp_lifecycle::LifecycleNode* node, const WorldState* world_state)
    : world_state_(world_state) {
    pub_ = node->create_publisher<MyMsg>("my_topic", 10);
    timer_ = node->create_wall_timer(100ms, [this]{ publish(); });
    timer_->cancel();  // Don't start until activate()
  }
  void activate() override { timer_->reset(); }
  void deactivate() override { timer_->cancel(); }
private:
  void publish() { /* read from world_state_.buffer<Car>().getAll() */ }
  WorldStateReader world_state_;
  // ...
};
```

### New Service

1. Create `include/world_model/interfaces/services/my_service.hpp`
2. Inherit from `InterfaceBase`
3. Create the service in the constructor — services are always available once configured
4. Register in `WorldModelNode::createInterfaces()`

### New Entity Type

1. Add to `EntityType` enum in `types/entity_type.hpp`
2. Create a subclass of `Entity3D` in `types/entity_3d.hpp`
3. Add an `EntityBuffer<NewType>` to `WorldState` with template specializations
4. Add classification mapping in `WorldModelWriter::classify()`
5. Add `runPipeline<NewType>(...)` call in `WorldModelWriter::onMessage()`
6. Add enrichment overload in `LaneletEnricher`
7. Add collection calls in publishers/services that iterate entity types (e.g., `DynamicObjectsPublisher::collectEntities<NewType>()`)
