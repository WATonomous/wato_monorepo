# Visualization Plugins

**Base class:** `eidos::VisualizationPlugin` (`include/eidos/plugins/base_visualization_plugin.hpp`)

Visualization plugins are read-only consumers of the optimized state. Each plugin runs on its own timer and callback group, independent of the SLAM loop. They read optimized values from an `AtomicSlot` (lock-free) and publish RViz-compatible messages.

## Pure Virtual Methods

```cpp
virtual void onInitialize() = 0;
virtual void render(const gtsam::Values& optimized_values) = 0;
```

## Virtual Methods

```cpp
virtual void onActivate();    // default: no-op
virtual void onDeactivate();  // default: no-op
```

## Lifecycle

1. `initialize()` sets up the plugin and calls `onInitialize()`.
2. On activation, a timer is started at the configured `publish_rate`. `onActivate()` is called.
3. Each timer tick reads the latest optimized values from the `AtomicSlot` and calls `render()`.
4. On deactivation, the timer is cancelled. `onDeactivate()` is called.

## Registration

1. Add the class to `visualization_plugins.xml`
2. Add `PLUGINLIB_EXPORT_CLASS(your_ns::YourPlugin, eidos::VisualizationPlugin)` at the bottom of the `.cpp`
3. List the plugin name in your config YAML under `visualization_plugins`

## Built-in Visualization Plugins

- [KeyframeMapVisualization](keyframe_map_visualization.md) -- Accumulated/windowed point cloud map
- [FactorGraphVisualization](factor_graph_visualization.md) -- RViz MarkerArray of the pose graph
