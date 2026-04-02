# MotionModelFactor

**Class:** `eidos::MotionModelFactor`
**Base class:** `eidos::FactorPlugin`
**Header:** `include/eidos/plugins/factors/motion_model_factor.hpp`

## Purpose

Bridges temporally consecutive SLAM states created by different factor plugins. When
two consecutive states in the graph are owned by different plugins (e.g., LISO creates
state N, then IMU creates state N+1), there is no direct constraint between them.
MotionModelFactor fills this gap by producing a `BetweenFactor<Pose3>` from a kinematic
prediction.

It does not subscribe to any sensor data. Instead, it calls `eidos_transform`'s
`PredictRelativeTransform` service, which uses its EKF-fused odometry to predict the
relative pose between two timestamps.

## How It Works

MotionModelFactor implements `latchFactor()` (not `produceFactor()`) -- it never creates
new states, only attaches constraints to states created by other plugins.

On each call to `latchFactor(key, timestamp)`:

1. If this is the first state seen, record it and return empty.
2. If dt < 0.01s since the last state, skip (timestamps too close).
3. Call `eidos_transform`'s `PredictRelativeTransform` service with `(timestamp_from, timestamp_to)`.
4. If the service responds successfully, construct a `BetweenFactor<Pose3>` between
   the previous state key and the current key, using the predicted relative pose.
5. Noise is scaled by dt -- longer temporal gaps produce less certain predictions.

## Parameters

| Parameter | Default | Description |
|---|---|---|
| `motion_model_factor.plugin` | `"eidos::MotionModelFactor"` | Plugin class name |
| `motion_model_factor.predict_service` | `"predict_relative_transform"` | Service name for `eidos_transform`'s PredictRelativeTransform |
| `motion_model_factor.noise_cov` | `[1e-2, 1e-2, 1e-2, 1e-4, 1e-4, 1e-4]` | Base noise covariance [rot3, trans3], scaled by dt at runtime |

## Configuration Example

```yaml
factor_plugins:
  - "liso_factor"
  - "gps_factor"
  - "motion_model_factor"

motion_model_factor:
  plugin: "eidos::MotionModelFactor"
  predict_service: "predict_relative_transform"
  noise_cov: [1.0e-2, 1.0e-2, 1.0e-2, 1.0e-4, 1.0e-4, 1.0e-4]
```

## Relationship to eidos_transform

This plugin depends on `eidos_transform` being running. If the predict service is not
available, the plugin logs a warning and skips the factor for that tick. No crash occurs
-- the graph simply lacks a between-factor for that state pair.

The `PredictRelativeTransform` service is provided by `eidos_transform`, which maintains
an EKF fusing multiple odometry sources. The service returns the EKF-predicted relative
pose between any two timestamps within its buffer window.

## Differences from the Old Motion Model

The old `MotionModelPlugin` (now removed from eidos) was a separate plugin type that:
- Ran inside eidos at 500 Hz for smooth TF prediction
- Was called directly by TransformManager for EKF fusion
- Provided `predict()` and `getBetweenFactor()` methods

The new `MotionModelFactor` is a standard `FactorPlugin` that:
- Runs at the SLAM tick rate (not 500 Hz)
- Delegates prediction to `eidos_transform` via a ROS service call
- Only produces `BetweenFactor<Pose3>` for the graph -- no TF involvement
