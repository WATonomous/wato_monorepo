# Factor Plugins

**Base class:** `eidos::FactorPlugin` (`include/eidos/plugins/base_factor_plugin.hpp`)

Factor plugins produce GTSAM factors from sensor data for the SLAM graph. Each plugin is self-contained: it manages its own subscriptions, publishers, and internal state. EidosNode does not broker raw data between plugins.

## Pure Virtual Methods

```cpp
virtual void onInitialize() = 0;
virtual void activate() = 0;
virtual void deactivate() = 0;
```

## Virtual Methods (override as needed)

```cpp
virtual StampedFactorResult produceFactor(gtsam::Key key, double timestamp);
virtual StampedFactorResult latchFactor(gtsam::Key key, double timestamp);
virtual bool isReady() const;  // default: true
virtual void onTrackingBegin(const gtsam::Pose3& pose);
virtual void onOptimizationComplete(const gtsam::Values& values, bool graph_corrected);
```

## Lifecycle

1. **`initialize()`** -- Called once by the framework. Sets `node_`, `tf_`, `callback_group_`, `map_manager_`, `estimator_pose_`, and `state_`, then calls `onInitialize()`.
2. **`onInitialize()`** -- Declare parameters, create subscriptions and publishers, register data formats with MapManager.
3. **`activate()` / `deactivate()`** -- Called on lifecycle transitions.
4. **`produceFactor(key, timestamp)`** -- Called every SLAM tick. Only state-creating plugins (e.g. LISO) override this. Set `result.timestamp` to signal a new state; return empty if no data.
5. **`latchFactor(key, timestamp)`** -- Called after a new state is created. Latching plugins (e.g. GPS, loop closure) attach constraints to states created by others.
6. **`isReady()`** -- Called during WARMING_UP to check if the plugin has completed any required warmup (e.g. IMU stationarity detection). Default returns `true`. The system stays in WARMING_UP until all factor plugins report ready.
7. **`onTrackingBegin(pose)`** -- Called once when TRACKING begins with the initial pose.
8. **`onOptimizationComplete(values, graph_corrected)`** -- Called after each ISAM2 optimization with the full optimized values and a flag indicating whether a significant graph correction occurred (loop closure or GPS correction).

## StampedFactorResult

The return type for both `produceFactor()` and `latchFactor()`:

| Field | Type | Description |
|---|---|---|
| `timestamp` | `std::optional<double>` | If set, EidosNode creates a new ISAM2 state at this timestamp. If nullopt, factors are latched onto an existing state. |
| `factors` | `vector<NonlinearFactor::shared_ptr>` | GTSAM factors to add to the graph. |
| `values` | `gtsam::Values` | Initial values for any new variables introduced (only used by state-creating plugins). |
| `loop_closure` | `bool` | When `true`, the optimizer runs extra iterations for loop closure convergence. |
| `correction` | `bool` | When `true` (and `loop_closure` is `false`), extra correction iterations run (e.g. GPS). |

## Lock-Free Pose Outputs

```cpp
// Protected (called by plugin from sensor callbacks):
void setMapPose(const gtsam::Pose3& pose);
void setOdomPose(const gtsam::Pose3& pose);

// Public (lock-free read, non-blocking):
std::optional<gtsam::Pose3> getMapPose() const;
std::optional<gtsam::Pose3> getOdomPose() const;
```

These are used for lock-free pose sharing within eidos. Not every factor plugin needs to set these -- only plugins that produce pose estimates (e.g. LISO).

## Registering a New Factor Plugin

1. Create a class inheriting `eidos::FactorPlugin`
2. Add the class to `factor_plugins.xml`:

```xml
   <library path="eidos_plugins">
     <class type="your_ns::YourPlugin" base_class_type="eidos::FactorPlugin">
       <description>Your plugin description</description>
     </class>
   </library>
   ```

1. Add `PLUGINLIB_EXPORT_CLASS(your_ns::YourPlugin, eidos::FactorPlugin)` at the bottom of the `.cpp`
2. List the plugin name in your config YAML under `factor_plugins`

## Registering Data Formats for Persistence

To persist plugin data in `.map` files, register formats in `onInitialize()`:

```cpp
map_manager_->registerKeyframeFormat("my_plugin/cloud", "pcl_pcd_binary");
map_manager_->registerGlobalFormat("my_plugin/calibration", "raw_double5");
```

Then store data per-keyframe via `map_manager_->store(key, "my_plugin/cloud", cloud_ptr)`.

## Built-in Factor Plugins

- [LisoFactor](liso_factor.md) -- LiDAR-Inertial Submap Odometry (GICP scan-to-submap matching)
- [GpsFactor](gps_factor.md) -- Unary GPS position constraints
- [EuclideanDistanceLoopClosureFactor](loop_closure_factor.md) -- KD-tree + GICP loop closure detection
- [ImuFactor](imu_factor.md) -- IMU preintegration factor
- [MotionModelFactor](motion_model_factor.md) -- Cross-plugin BetweenFactor bridging via eidos_transform predict service
