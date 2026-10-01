# OdometryFactor

**Class:** `eidos::OdometryFactor`
**Type:** Factor (latching via `latchFactor`)
**XML:** `factor_plugins.xml`

A relative-pose constraint from any external odometry source that publishes `nav_msgs/Odometry`. It was written for the cuVSLAM visual odometry in `src/perception/visual_odometry`, and it works for others such as wheel odometry.

The plugin buffers the source's poses. For every pair of consecutive states created by another plugin (usually LISO), it adds a `BetweenFactor<Pose3>` holding the source's motion between the two state timestamps. That motion is interpolated at those timestamps and re-expressed in eidos' body frame.

## How It Works

1. **Buffer.** The subscription pushes `(header.stamp + time_offset, pose)` into a time-indexed buffer that holds `buffer_duration` seconds.
   - On the first message, the extrinsic `frames.base_link` <- `child_frame_id` is looked up in TF.
   - For the pano camera rig this is `base_footprint` <- `base_link`, which is +1.76 m in z.
2. **Queue.** `latchFactor(key, t)` pairs each new state with the previous one and queues the pair.
3. **Wait.** A queued pair is measured only when both of these hold:
   - the buffer covers its end time. The source lags the keyframe sensor: camera transport plus VO processing is about 50-80 ms after the lidar stamp.
   - both states have an optimized estimate. This is needed for gating.

   In practice a pair is delivered on the **next** `latchFactor()` call, carrying a factor between two existing states. This is the same deferred delivery the loop closure plugin uses. It does not use the `produceFactor` standalone-factor path.
4. **Measure.** The source's relative motion `T(t_a)^-1 T(t_b)` is interpolated (SLERP for rotation, linear for translation) and conjugated into the body frame: `T_body_child * rel * T_body_child^-1`.
   - If any gap between buffered samples inside `[t_a, t_b]` is longer than `max_gap`, no factor is produced. Such a gap means the source lost tracking or reset its frame.
5. **Gate.** The measurement is compared with the graph's current relative pose between the two states, which is dominated by LISO.
   - The residual `rel_measured^-1 * rel_graph` is published on `<name>/residual`.
   - The pair is rejected if the residual exceeds `gate_trans` / `gate_rot`.
6. **Factor.** A `BetweenFactor<Pose3>` is added with `Diagonal::Sigmas([σr×3, σt×3])`, wrapped in a Huber or Cauchy robust kernel.
   - The sigmas grow with the keyframe distance `d`: `σ = sigma + sigma_per_m · d`.
   - With `add_factors: false` (shadow mode), everything except adding the factor still happens, including the residuals and counters.

Pairs the odometry has not covered within `max_pending_age` seconds (of state time) are dropped. A throttled log line reports counts of emitted, gated, ungated, gap, out-of-range and expired pairs.

## Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `odom_topic` | string | `"/perception/visual_odometry/odometry"` | Input `nav_msgs/Odometry`. Only `header.frame_id` of the first message is accepted. Nothing should be published while the source is lost. |
| `add_factors` | bool | `true` | `false` = shadow mode: measure, gate and publish residuals, but add no factors. |
| `time_offset` | double | `0.0` | Seconds added to message stamps. The pano cameras stamp at host arrival, not exposure. Estimate the offset with `tools/vo_eval.py --imu`. |
| `max_gap` | double | `0.15` | A gap longer than this (s) between buffered samples inside a pair means a reset, so the pair gets no factor. Use about 1.5 × the source period; the VO runs at 20 Hz, so 0.075. |
| `max_pending_age` | double | `3.0` | Drop a pair the odometry has not covered after this many seconds of newer state time. |
| `buffer_duration` | double | `30.0` | Seconds of odometry history kept. |
| `rot_sigma` | double | `0.005` | Rotation sigma floor (rad). |
| `rot_sigma_per_m` | double | `0.002` | Added rotation sigma per metre between the states (rad/m). |
| `trans_sigma` | double | `0.05` | Translation sigma floor (m). |
| `trans_sigma_per_m` | double | `0.03` | Added translation sigma per metre between the states. |
| `robust_kernel` | string | `"huber"` | `none`, `huber` or `cauchy`. |
| `robust_k` | double | `1.345` | Robust kernel parameter. |
| `gate_trans` | double | `1.0` | Reject a pair whose translation disagrees with the graph by more than this (m). `0` disables the check. |
| `gate_rot` | double | `0.1` | Reject a pair whose rotation disagrees with the graph by more than this (rad). `0` disables the check. |

Reads the global `frames.base_link` (eidos' body frame, e.g. `base_footprint`).

## Topics

| Topic | Type | Description |
|---|---|---|
| `<odom_topic>` (sub) | `nav_msgs/Odometry` | External odometry. |
| `<name>/residual` (pub) | `geometry_msgs/PoseStamped` | Per measured pair: `rel_measured^-1 * rel_graph`. Stamped at the later state and expressed in `frames.base_link`. |

## Notes

- Only relative motion is used. The source's frame origin, its absolute pose and its message covariance are ignored. cuVSLAM's covariance is absolute and, for small rigs, uninformative.
- Gating compares against the graph, which is mostly LISO. When LISO itself is degenerate, a correct VO measurement can be gated. In that case set `gate_trans`/`gate_rot` to `0` and rely on the robust kernel.
- The first time the source publishes, TF must provide `frames.base_link` <- `child_frame_id`. Until it does, messages are dropped with a throttled warning.
- If the source's stamps jump backwards by more than 1 s (a bag loop or clock reset), the buffer restarts.

## Mapping vs Localization

| Parameter | Mapping | Localization |
|---|---|---|
| `add_factors` | `true` | `true` (relative constraints between the tracked states) or `false` for shadow evaluation |

## Configuration Example

```yaml
factor_plugins:
  - "gps_factor"
  - "liso_factor"
  - "visual_odometry_factor"

visual_odometry_factor:
  plugin: "eidos::OdometryFactor"
  odom_topic: "/perception/visual_odometry/odometry"
  max_gap: 0.075
  add_factors: false   # shadow mode first: inspect visual_odometry_factor/residual
```
