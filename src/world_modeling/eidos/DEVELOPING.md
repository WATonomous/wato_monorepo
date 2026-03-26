# Eidos Developer Guide

## 1. System Overview

Eidos is a plugin-based SLAM and localization system built as a ROS 2 lifecycle node.
It uses GTSAM's ISAM2 for incremental factor graph optimization. Plugins provide
sensor-specific factors, relocalization strategies, and visualization. Eidos is
pure SLAM -- it does not broadcast TF or run an EKF. TF broadcasting and odometry
fusion are handled by the separate `eidos_transform` package.

```
                         EidosNode (LifecycleNode)
                         ========================
                         |                      |
        tick() @ slam_rate_ Hz          services: save_map / load_map
                         |
          +--------------+----------------+
          |              |                |
    InitSequencer    Estimator      PluginRegistry
    (state machine)  (ISAM2 engine) (owns all plugins + ClassLoaders)
          |              |                |
          |              |     +----------+----------+-----------+
          |              |     |          |                      |
          |              |   Factor    Relocalization      Visualization
          |              |   Plugins   Plugins             Plugins
          |              |
          |            MapManager
          |            (keyframe poses + data,
          |             SQLite persistence)

Data flow (per SLAM tick):
  Sensor callbacks --> FactorPlugin buffers (lock-free)
  SLAM tick        --> poll plugins for factors --> Estimator.optimize()
  Estimator        --[LockFreePose]--> published slam/pose topic
  Estimator        --[AtomicSlot<Values>]--> VisualizationPlugins (reads lock-free)
  Estimator        --> MapManager.updatePoses()
```

## 2. Core Components

### EidosNode

Top-level lifecycle node that owns all core components: `Estimator`, `InitSequencer`,
`MapManager`, and `PluginRegistry`. It drives the SLAM loop via a timer at `slam_rate_`
Hz. Each tick delegates to `InitSequencer` until `TRACKING` is reached, then calls
`handleTracking()` which polls factor plugins, feeds factors to `Estimator`, and
publishes pose/odometry. Exposes `save_map` and `load_map` ROS services. Shares a
`tf2_ros::Buffer` with plugins for extrinsic lookups (static transforms from URDF).

Does not broadcast TF. Publishes `slam/pose` which `eidos_transform` subscribes to
for TF updates.

### Estimator

ISAM2 optimization engine. Has no orchestration logic -- `EidosNode` tells it when to
optimize. Maintains a monotonic state timeline (`Symbol('x', N)`). After each
`optimize()` call, it writes the latest optimized pose to a `LockFreePose` and the
full optimized `Values` to an `AtomicSlot` (read by visualization plugins). Provides
`optimizeExtra()` for additional iterations after loop closure or GPS corrections. Key
methods: `configure()`, `reset()`, `optimize()`, `optimizeExtra()`, `createState()`,
`getPose()`, `getCovariance()`.

### MapManager

Centralized keyframe pose and data store. At runtime, all per-keyframe and global data
lives in memory as `std::any`, accessed via typed `store<T>()`/`retrieve<T>()` templates.
Manages keyframe 3D/6D point clouds and a KD-tree for spatial queries. Tracks graph
adjacency (edges between keyframes). For persistence, uses a single SQLite `.map` file.
Plugins register data formats via `registerKeyframeFormat()` and `registerGlobalFormat()`,
linking data keys to named format handlers from the shared format registry. Key methods:
`addKeyframe()`, `updatePoses()`, `store<T>()`/`retrieve<T>()`,
`storeGlobal<T>()`/`retrieveGlobal<T>()`, `saveMap()`, `loadMap()`.

### PluginRegistry

Loads, owns, and provides lookup for all plugins. Uses `pluginlib::ClassLoader` for
each plugin category: factor plugins, relocalization plugins, and visualization plugins.
Loading happens once during `EidosNode::on_configure()`. After loading, the registry
is read-only. Other components hold `const` pointers for lock-free plugin lookups.
Plugin collections are public for read access: `factor_plugins`, `reloc_plugins`,
`vis_plugins`. Provides `findFactor(name)` for named lookup.

### InitSequencer

Manages the `INIT -> WARMING_UP -> RELOCALIZING -> TRACKING` state machine, separated
from `EidosNode` so orchestration logic stays clean. `EidosNode` calls `step()` each
SLAM tick. In `WARMING_UP`, if a prior map is loaded it transitions to `RELOCALIZING`
and polls relocalization plugins. If no prior map, it transitions directly to `TRACKING`
with an identity pose. Once a relocalization result is obtained (or the timeout expires),
it invokes the `on_tracking` callback which triggers `EidosNode::beginTracking()`. Only
called from the SLAM loop thread.

## 3. Threading Model

Eidos uses a multi-threaded executor. Each component has its own callback group to
allow concurrent execution where safe.

### SLAM Loop (configurable, typically 10 Hz)

- Own `MutuallyExclusive` callback group (`slam_callback_group_`).
- The `tick()` timer fires at `slam_rate_` Hz.
- Processes all factor plugins synchronously in a single tick (polls each for factors,
  feeds to Estimator, runs ISAM2 optimization).
- Writes to `LockFreePose` and `AtomicSlot<Values>` after optimization.
- Calls `MapManager::updatePoses()` and `MapManager::addKeyframe()`.
- Publishes `slam/pose` and `slam/odometry`.

### Visualization Plugins (typically 1 Hz each)

- Each visualization plugin has its own callback group and timer.
- Reads `AtomicSlot<Values>` from Estimator (pointer-swap read).
- Reads from `MapManager` (mutex-protected).
- Independent of SLAM loop timing.

### Plugin Sensor Callbacks

- Each plugin has its own callback group for ROS subscriptions.
- Sensor data arrives concurrently with the SLAM loop.
- Plugins buffer data internally and expose it via lock-free getters
  (e.g., `getOdomPose()` returns from a `LockFreePose`).
- The SLAM loop polls plugins via `produceFactor()` without contention.

```
Thread 1 (SLAM loop @ 10 Hz):
  tick() -> poll plugins -> Estimator::optimize()
         -> LockFreePose.store()
         -> AtomicSlot<Values>.store()   --> [read by Thread 2..N]
         -> MapManager::updatePoses()
         -> publish slam/pose, slam/odometry

Thread 2..N (Visualization plugins @ 1 Hz each):
  timer -> AtomicSlot<Values>.load() [pointer swap]
        -> MapManager reads [mutex]
        -> publish

Thread M..Z (Plugin sensor callbacks):
  subscription callback -> buffer data internally
                        -> LockFreePose.store() for odom/map pose
```

## 4. Lock-Free Data Flow

### LockFreePose (seqlock)

Single-writer, multi-reader lock-free storage for `gtsam::Pose3` (~100 bytes).
Uses a seqlock pattern: the writer increments an atomic sequence counter to odd before
writing and back to even after. A reader that observes an odd counter (writer active)
retries. Since the copy takes nanoseconds, readers effectively never block.

Usage points:
- **Estimator -> EidosNode**: Estimator writes `optimized_pose_` after each
  `optimize()` call. EidosNode reads it to publish `slam/pose`.
- **Plugin -> EidosNode**: Factor plugins write their odom-frame and map-frame poses
  from sensor callbacks via `setOdomPose()`/`setMapPose()`. These are available for
  internal pose sharing between plugins within eidos (e.g., for cross-plugin state
  bridging via `MotionModelFactor`).

### AtomicSlot (pointer swap)

Single-writer, multi-reader slot for large payloads. The writer creates a new
`shared_ptr<const T>` and swaps it in under a brief mutex. Readers get the latest
snapshot as a `shared_ptr<const T>`.

Usage points:
- **Estimator -> VisualizationPlugins**: Estimator writes `optimized_values_`
  (full `gtsam::Values`) after each optimization. Visualization plugins read via
  `load()` to render the pose graph.

### Plugin Pose Slots

Factor plugins expose `getOdomPose()` and `getMapPose()` returning
`std::optional<gtsam::Pose3>` via their own `LockFreePose` instances. The plugin's
sensor callback thread writes. No mutex contention on the hot path. These are used
for lock-free pose sharing within eidos -- not for TF broadcasting (which is handled
by `eidos_transform`).

## 5. State Machine

```
  +----------------+
  | INITIALIZING   |  Node created, not yet configured/activated.
  +-------+--------+
          |
          |  on_activate() -> InitSequencer::reset()
          v
  +----------------+
  | WARMING_UP     |  Checking for prior map.
  +-------+--------+
          |
          +--- no prior map ---> on_tracking(Identity)
          |
          +--- prior map loaded:
          v
  +----------------+
  | RELOCALIZING   |  Polling relocalization plugins each tick.
  +-------+--------+
          |
          +--- reloc plugin returns a pose ---> on_tracking(reloc_pose)
          |
          +--- timeout (configurable) -------> on_tracking(Identity)
          v
  +----------------+
  | TRACKING       |  SLAM loop active. Estimator running.
  +----------------+
```

**Transition triggers:**

- `INITIALIZING -> WARMING_UP`: `EidosNode::on_activate()` calls `InitSequencer::reset()`.
- `WARMING_UP -> TRACKING`: No prior map is loaded. `on_tracking` callback is invoked
  with `Pose3::Identity()`.
- `WARMING_UP -> RELOCALIZING`: A prior map is loaded (`map_manager_->hasPriorMap()`).
- `RELOCALIZING -> TRACKING`: A relocalization plugin returns a valid pose via
  `tryRelocalize()`, or the relocalization timeout expires (falls back to identity pose).
  `on_tracking` callback is invoked with the resulting pose.

The `on_tracking` callback triggers `EidosNode::beginTracking()`, which resets the
Estimator and notifies all factor plugins via `onTrackingBegin()`.

## 6. Map Persistence

Maps are stored as single SQLite `.map` files. The file is self-describing: a
`data_formats` table records which serialization format each data key uses, so external
tools (e.g., `eidos_tools` CLI) can deserialize without external knowledge.

### SQLite Schema

```sql
-- Key-value metadata (version, num_states, etc.)
CREATE TABLE metadata (
  key   TEXT PRIMARY KEY,
  value TEXT
);

-- Keyframe poses (6-DOF + timestamp + owner plugin)
CREATE TABLE keyframes (
  id        INTEGER PRIMARY KEY,
  gtsam_key INTEGER UNIQUE,
  x REAL, y REAL, z REAL,
  roll REAL, pitch REAL, yaw REAL,
  time REAL,
  owner TEXT
);

-- Per-keyframe binary data blobs (point clouds, descriptors, etc.)
CREATE TABLE keyframe_data (
  gtsam_key INTEGER,
  data_key  TEXT,
  data      BLOB,
  PRIMARY KEY (gtsam_key, data_key)
);

-- Global binary data blobs (e.g., SC matrix for loop closure)
CREATE TABLE global_data (
  data_key TEXT PRIMARY KEY,
  data     BLOB
);

-- Factor graph edges between keyframes
CREATE TABLE edges (
  key_a INTEGER,
  key_b INTEGER,
  owner TEXT,
  PRIMARY KEY (key_a, key_b)
);

-- Self-describing format registry (maps data keys to format handlers)
CREATE TABLE data_formats (
  data_key TEXT PRIMARY KEY,
  format   TEXT,
  scope    TEXT   -- "keyframe" or "global"
);
```

### Format Registry Pattern

Serialization is handled by a shared format registry (`eidos::formats::registry()`).
Each format is a subclass of `eidos::formats::Format` providing `serialize(std::any) ->
bytes` and `deserialize(bytes) -> std::any`. The registry is a global map from format
name (e.g., `"pcl_pcd_binary"`) to `Format` instance. Both `eidos` and `eidos_tools`
link the same registry.

### How Plugins Register Formats

During initialization, each plugin that persists data calls:

```cpp
map_manager->registerKeyframeFormat("my_plugin/cloud", "pcl_pcd_binary");
map_manager->registerGlobalFormat("my_plugin/global_descriptor", "eigen_binary");
```

This tells `MapManager` that when saving, the `std::any` stored under key
`"my_plugin/cloud"` should be serialized using the `"pcl_pcd_binary"` format handler.
The `data_formats` table records this mapping so that loading can find the correct
deserializer without any compile-time coupling to specific plugins.

**Type contract**: The `std::any` stored via `store()` for a given `data_key` must
contain the C++ type that the named format expects (e.g., `"pcl_pcd_binary"` expects
`pcl::PointCloud<PointType>::Ptr`). A mismatch causes `std::bad_any_cast` at save time,
which is caught and logged.

### Save / Load Flow

- **Save** (`saveMap(path)`): Opens a new SQLite file. Writes metadata, keyframe poses,
  serialized keyframe data blobs, global data blobs, edges, and format mappings in a
  single transaction. Uses WAL journal mode.
- **Load** (`loadMap(path)`): Opens the SQLite file read-only. Clears existing state.
  Loads keyframe poses immediately. Reads the `data_formats` table to discover which
  format handles each data key, then deserializes all keyframe and global data blobs
  using the format registry. Loaded keyframes are tracked as prior map keys
  (`prior_map_keys_`) so the system can distinguish them from newly created keyframes.
