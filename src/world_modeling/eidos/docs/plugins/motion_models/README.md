# Motion Model Plugins

**Base class:** `eidos::MotionModelPlugin` (`include/eidos/plugins/base_motion_model_plugin.hpp`)

A motion model is a **kinematic state transition model**. It predicts the next pose given the current pose and a time step. It does NOT subscribe to raw sensor data — sensor measurements come from factor plugins (LISO, GPS, ImuFactor).

TransformManager calls `predict()` at 500 Hz for smooth `odom -> base_link` TF. After each measurement correction (e.g. from LISO), TransformManager calls `onMeasurementUpdate()` so the model can update its internal velocity estimate.

Motion models can also bridge gaps between consecutive states created by different factor plugins via `getBetweenFactor()`.

Only one motion model is loaded per configuration.

## Pure Virtual Methods

```cpp
virtual void onInitialize() = 0;
virtual void activate() = 0;
virtual void deactivate() = 0;
virtual gtsam::Pose3 predict(const gtsam::Pose3& current, double dt) = 0;
```

## Virtual Methods

```cpp
virtual void onMeasurementUpdate(const gtsam::Pose3& corrected, double timestamp);  // default: no-op
virtual gtsam::NonlinearFactor::shared_ptr getBetweenFactor(
    gtsam::Key key_from, double ts_from,
    gtsam::Key key_to, double ts_to);  // default: nullptr
virtual bool isReady() const;          // default: true
```

`predict()` is the core method. TransformManager calls it every tick with the current EKF state and the time step. The model returns the predicted next pose.

`onMeasurementUpdate()` is called after each measurement correction so the model can estimate velocity from consecutive corrected poses.

`isReady()` gates the WARMING_UP -> RELOCALIZING/TRACKING transition.

## Registration

1. Add the class to `motion_model_plugins.xml`
2. Add `PLUGINLIB_EXPORT_CLASS(your_ns::YourPlugin, eidos::MotionModelPlugin)` at the bottom of the `.cpp`
3. Reference the plugin class in your config YAML under `motion_model.plugin`

## Built-in Motion Models

- [HolonomicMotionModel](holonomic_motion_model.md) -- Constant-velocity state transition (coasts between corrections)
- [AckermannMotionModel](ackermann_motion_model.md) -- Ackermann kinematics (steering + velocity)
