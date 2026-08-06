# fault_collector

Central aggregator for fault reports (`wato_fault_msgs/Fault`) published by any node in the monorepo. Turns those reports into a `diagnostic_msgs/DiagnosticArray` for Foxglove's Diagnostics panel and two independent, severity-driven response tiers.

## Overview

Any node that detects a problem -- a stuck sensor, a failed calibration, a solver that stopped converging -- publishes a `Fault` message on `/faults` describing what broke, how severe it is, and where it came from. `fault_collector` is the single place that turns that stream of reports into diagnostics plus real vehicle response, split into two independent tiers:

- **Tier A -- Disarm** (`disarm_level`, default 1 = CRITICAL only): calls `oscc_interfacing`'s `/oscc_interfacing/arm` service (`std_srvs/SetBool`) with `data: false` on the rising edge into "disarm required". `oscc_interfacing` is the real vehicle authority -- it already ignores every roscco command while disarmed and performs its own graceful ~600ms steering-torque rampdown. **`fault_collector` never modifies `oscc_interfacing`.**
- **Tier B -- Brake** (`brake_level`, default 2 = SEVERE and worse): asserts a latched `emergency_stop` Bool that `ackermann_mux` treats as a hard override, continuously, for as long as the condition holds -- not edge-triggered like Tier A.
- Fault levels below both thresholds show up in `/diagnostics` only; no actuation.

`disarm_level` must be `<= brake_level`: a disarm-worthy fault is always also brake-worthy, never the reverse.

**Re-engage lockout.** `oscc_interfacing` itself places no lockout on its own arm service -- it will happily re-arm the instant it gets a `SetBool(true)`, regardless of why it was last disarmed. Since `oscc_interfacing` is deliberately never modified, `fault_collector` enforces the lockout entirely on its own side: it subscribes to `oscc_interfacing`'s `is_armed` topic, and every time that transitions `false -> true` while a disarm-level fault is still active or latched, it immediately calls the arm service again with `data: false`. In other words, while `latch: true` (the default) and a disarm-level fault has ever fired, the operator's arm button is put right back down the instant it's pressed. Repeated re-disarm attempts are rate-limited by `min_disarm_interval_s` and each one is logged at `RCLCPP_ERROR` naming the reason. The only way out is `~/clear_faults` -- and even that only clears the *latch*; if the offending fault's source is still actively reporting it, it simply re-populates the table and re-trips on the very next report (see the service's response message).

**Safety fallback:** if a Tier A disarm attempt fails -- the arm service isn't reachable, or it responds `success: false` -- `fault_collector` forces `emergency_stop` true regardless of `brake_level`, and that fallback persists until `~/clear_faults` is called. If the vehicle cannot be confirmed disarmed, it must at least be braked.

## Architecture

```
                                       ┌─► /diagnostics                            (Foxglove)
                                       │
any node ──Fault──► /faults ──► fault_collector
                                  │    │
                                  │    ├─► SetBool(false) ───────► /oscc_interfacing/arm   (Tier A: real vehicle
                                  │    │   (rising edge, and                                authority, unmodified)
                                  │    │    on every re-engage             ▲
                                  │    │    while disarm required)         │
                                  │    │                                   │ is_armed (Bool)
                                  │    │   ┌───────────────────────────────┘  re-engage lockout: re-disarm on
                                  │    └───┤   /oscc_interfacing/is_armed      false->true while disarm required
                                  │        └───────────────────────────────────────────────────────
                                  │
                                  ├─► /faults/autonomy_disarm  (status only, for observability)
                                  │
                                  └─► /faults/emergency_stop ──► ackermann_mux            (Tier B: hard override,
                                      (latched Bool, continuous)                          except bypass_inputs)
```

On each publish tick (`publish_rate_hz`) the node:
1. Expires any active fault that has not been re-reported within `fault_timeout_s`.
2. Builds a `DiagnosticArray`: one summary status plus one status per active fault.
3. If Tier A's disarm condition just rose (edge), calls `arm_service` and logs which fault(s) triggered it.
4. Publishes `autonomy_disarm` (Tier A status, observability only) and `emergency_stop` (Tier B, the real override signal `ackermann_mux` acts on; forced `true` if the last disarm attempt failed).

Independently of the publish tick, the `is_armed` callback fires the same disarm request whenever the vehicle is re-armed while disarm is still required -- that is the event-driven half of the re-engage lockout, and it is *not* on a timer.

**Latching**: with `latch: true` (the default), once a tier trips it stays tripped even if the offending fault clears or expires on its own, until `~/clear_faults` is called. This applies independently to Tier A and Tier B, and for Tier A it is exactly what makes the re-engage lockout meaningful -- without latching, the lockout would only apply while the fault is still literally being reported.

## Publishing a fault from your node

**C++:**

```cpp
#include "wato_fault_msgs/msg/fault.hpp"

auto fault_pub = node->create_publisher<wato_fault_msgs::msg::Fault>("/faults", rclcpp::QoS(rclcpp::KeepLast(10)));

wato_fault_msgs::msg::Fault fault;
fault.header.stamp = node->now();
fault.level = wato_fault_msgs::msg::Fault::SEVERE;
fault.source = "lidar_driver";
fault.message = "Point cloud stream stalled";
fault.id = "stalled_stream";  // stable id; omit to key on `message` instead
fault.active = true;
fault_pub->publish(fault);

// ... later, to clear it explicitly:
fault.active = false;
fault_pub->publish(fault);
```

**`ros2 topic pub`:**

```bash
ros2 topic pub /faults wato_fault_msgs/msg/Fault \
  "{level: 1, source: 'test_bench', message: 'manual critical fault', id: 'manual_test', active: true}" --once
```

Re-publish the same `(source, id)` periodically to keep the fault alive; stop publishing and it expires after `fault_timeout_s`, or publish once more with `active: false` to clear it immediately.

## Build & Launch

```bash
colcon build --packages-select fault_collector
ros2 launch fault_collector fault_collector.launch.yaml
```
