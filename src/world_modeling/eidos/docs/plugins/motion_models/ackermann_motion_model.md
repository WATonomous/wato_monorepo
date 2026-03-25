# AckermannMotionModel

**Class:** `eidos::AckermannMotionModel`
**XML:** `motion_model_plugins.xml`

Ackermann kinematic state transition model. Subscribes to `TwistStamped` (linear.x = speed, angular.z = steering angle) and uses ackermann kinematics to predict the next pose:

```
dx   = speed * dt
dyaw = speed * tan(steering_angle) / wheelbase * dt
```

When no data is available, prediction returns the current pose (zero velocity).

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `twist_topic` | string | `"ackermann_twist"` | Input TwistStamped topic (linear.x = speed, angular.z = steering angle). |
| `wheelbase` | double | `2.57` | Vehicle wheelbase in meters. |

## Notes

- `isReady()` returns true immediately. If no ackermann data arrives, prediction is identity (no motion).
- The upstream `TwistStamped` publisher (from CAN state estimator) is not yet implemented. This model is a stub until that publisher exists.
- `onMeasurementUpdate()` is not overridden — ackermann uses direct steering/velocity input rather than inferring velocity from corrections.

## Mapping vs Localization

No behavioral difference. The ackermann model predicts from steering + velocity regardless of mode.
