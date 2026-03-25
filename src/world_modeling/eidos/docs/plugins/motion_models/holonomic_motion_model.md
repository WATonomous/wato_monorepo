# HolonomicMotionModel

**Class:** `eidos::HolonomicMotionModel`
**XML:** `motion_model_plugins.xml`

Holonomic constant-velocity state transition model. Maintains a 6-DOF velocity estimate (vx, vy, vz, wx, wy, wz in body frame) updated from consecutive measurement-corrected poses. Between measurement corrections (~20Hz from LISO), the model coasts at the last known velocity for smooth 500Hz TF output.

No external subscriptions. Velocity is inferred entirely from measurement corrections via `onMeasurementUpdate()`.

## Parameters

This plugin declares no parameters.

## Notes

- `predict()` propagates `current.compose(Expmap(velocity * dt))` — constant-velocity in body frame.
- `onMeasurementUpdate()` estimates velocity from `Logmap(prev_corrected.between(corrected)) / dt`.
- `isReady()` returns true immediately. No warmup needed.
- This is the default motion model for eidos.

## Mapping vs Localization

No behavioral difference. The holonomic model is identical in both modes — it coasts between whatever measurement corrections the factor plugins provide.
