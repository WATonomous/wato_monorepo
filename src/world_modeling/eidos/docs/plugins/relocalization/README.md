# Relocalization Plugins

**Base class:** `eidos::RelocalizationPlugin` (`include/eidos/plugins/base_relocalization_plugin.hpp`)

Relocalization plugins determine the initial pose against a prior map during the RELOCALIZING state. Multiple relocalization plugins can be loaded; the first to return a successful result wins.

## Pure Virtual Methods

```cpp
virtual void onInitialize() = 0;
virtual void activate() = 0;
virtual void deactivate() = 0;
virtual std::optional<RelocalizationResult> tryRelocalize(double timestamp) = 0;
```

## RelocalizationResult

```cpp
struct RelocalizationResult {
  gtsam::Pose3 pose;
  double fitness_score;
  int matched_keyframe_index;
};
```

## Lifecycle

1. `tryRelocalize()` is called repeatedly during the RELOCALIZING state (at `slam_rate`).
2. The first plugin to return a non-empty result provides the initial pose.
3. If all plugins return `std::nullopt` for `relocalization_timeout` seconds, the system falls back to starting from the origin.

## Registration

1. Add the class to `relocalization_plugins.xml`
2. Add `PLUGINLIB_EXPORT_CLASS(your_ns::YourPlugin, eidos::RelocalizationPlugin)` at the bottom of the `.cpp`
3. List the plugin name in your config YAML under `relocalization_plugins`

## Built-in Relocalization Plugins

- [GpsIcpRelocalization](gps_icp_relocalization.md) -- GPS coarse + GICP fine alignment against prior map
