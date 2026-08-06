# Developing fault_collector

## Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `faults` (remapped to `/faults`) | `wato_fault_msgs/Fault` | Fault reports from any node in the monorepo |

### Published

| Topic | Type | Description |
|-------|------|-------------|
| `diagnostics_topic` (default `/diagnostics`, absolute) | `diagnostic_msgs/DiagnosticArray` | One summary status + one status per active fault, at `publish_rate_hz` |
| `emergency_stop_topic` (remapped to `/faults/emergency_stop`) | `std_msgs/Bool` | Tier B: latched (transient-local) brake signal consumed by `ackermann_mux` |
| `autonomy_disarm_topic` (remapped to `/faults/autonomy_disarm`) | `std_msgs/Bool` | Tier A status, for observability/Foxglove only -- the real action is the service call below |

### Services / Clients

| Service | Type | Direction | Description |
|---------|------|-----------|--------------|
| `~/clear_faults` | `std_srvs/Trigger` | Server | Drops every active fault, releases both latches and the disarm edge detector, and clears the disarm-failure fallback flag. The only way out of a latched tier. |
| `arm_service` (default `/oscc_interfacing/arm`) | `std_srvs/SetBool` | Client | Real vehicle authority's arm service. Called with `data: false`, once per Tier A rising edge. **`oscc_interfacing` is never modified by this package.** |

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `faults_topic` | string | `"faults"` | Input fault topic (remapped to `/faults` in launch) |
| `diagnostics_topic` | string | `"/diagnostics"` | Output diagnostics topic (absolute) |
| `emergency_stop_topic` | string | `"emergency_stop"` | Tier B output topic (remapped to `/faults/emergency_stop`) |
| `autonomy_disarm_topic` | string | `"autonomy_disarm"` | Tier A status output topic (remapped to `/faults/autonomy_disarm`) |
| `arm_service` | string | `"/oscc_interfacing/arm"` | `std_srvs/SetBool` service called to disarm the vehicle |
| `publish_rate_hz` | double | `5.0` | Rate at which `/diagnostics`, `emergency_stop`, and `autonomy_disarm` are published |
| `fault_timeout_s` | double | `5.0` | A fault not re-reported within this window expires automatically |
| `disarm_level` | int | `1` | Tier A: faults at or below this level request a disarm (must be `<= brake_level`) |
| `brake_level` | int | `2` | Tier B: faults at or below this level request an emergency brake |
| `latch` | bool | `true` | Once a tier trips, stay tripped until `~/clear_faults` is called (both tiers, independently) |
| `disarm_enabled` | bool | `true` | Whether Tier A actually calls `arm_service` on a rising edge (false = observe only) |
| `diagnostic_name_prefix` | string | `"fault_collector"` | Prefix for `DiagnosticStatus.name` |
| `diagnostic_level_map` | int[4] | `[2, 2, 1, 0]` | Fault level 1..4 (CRITICAL..MILD) -> `DiagnosticStatus` level (OK=0, WARN=1, ERROR=2, STALE=3) |

## Example Configuration

```yaml
/**/fault_collector:
  ros__parameters:
    faults_topic: "faults"
    diagnostics_topic: "/diagnostics"
    emergency_stop_topic: "emergency_stop"
    autonomy_disarm_topic: "autonomy_disarm"
    arm_service: "/oscc_interfacing/arm"
    publish_rate_hz: 5.0
    fault_timeout_s: 5.0
    disarm_level: 1
    brake_level: 2
    latch: true
    disarm_enabled: true
    diagnostic_name_prefix: "fault_collector"
    diagnostic_level_map: [2, 2, 1, 0]
```

## Build & Launch

```bash
colcon build --packages-select fault_collector
ros2 launch fault_collector fault_collector.launch.yaml
```

## Internal Architecture

The node is a lifecycle node. Logic is split into `FaultCollectorCore` (pure C++, no ROS types, no clock access -- every method takes `now_s` explicitly) and `FaultCollectorNode` (ROS plumbing: subscription, timer, publishers, service, and the `arm_service` client). This split is what makes the arbitration logic unit-testable without spinning a node -- see `test/test_fault_collector_core.cpp`.

**Fault table keying:** faults are keyed by `source + "|" + (id.empty() ? message : id)`. Reporting the same key again bumps `count` and refreshes `last_seen_s` while preserving `first_seen_s`. Different sources reporting the same `message` are always separate entries, because `source` is always part of the key.

**Expiry vs explicit clear:** a fault leaves the table two ways -- it is not re-reported within `fault_timeout_s` (checked once per publish tick, before diagnostics are built), or its source publishes the same `(source, id)` again with `active: false`. Both are logically "the fault is gone," but only an explicit `active: false` is instantaneous; expiry has up to one `fault_timeout_s` of lag by design.

**How the re-engage lockout works.** `oscc_interfacing` is the real vehicle authority: it owns arming state, ignores every roscco command while disarmed, and does its own graceful rampdown on disarm. It places **no lockout** on its own arm service -- it will re-arm the instant it receives `SetBool(true)`, regardless of why it was last disarmed. A persistent fault must not be re-engageable, so `fault_collector` enforces the lockout entirely on its own side, in two parts:

1. **Edge-triggered request.** `FaultCollectorCore::consume_disarm_edge()` returns `true` exactly once per rising edge into "disarm requested", and the node calls `arm_service` on that single `true`. This keeps service traffic bounded -- the disarm is not re-sent every publish tick.
2. **Re-arm interception.** The node subscribes to `oscc_interfacing`'s `is_armed` topic. On every `false -> true` transition, it checks `FaultCollectorCore::disarm_required()` -- a deliberately **non-mutating** query (it neither sets the latch nor consumes the edge, since it is polled from a callback that can fire at any time) -- and if a disarm-level fault is still active or latched, immediately re-issues the disarm. The operator's arm button is put straight back down.

Repeated re-disarm attempts are rate-limited by `min_disarm_interval_s` so an arm/disarm fight cannot flood the service, and each is logged at `RCLCPP_ERROR` with the reason.

Note that the latches are set in `report()` itself, not as a side effect of polling `should_disarm()`/`should_brake()`. That matters: `disarm_required()` is read from the `is_armed` callback, which can fire before the next diagnostics tick, so the lockout must not depend on any getter having been called first.

The way out is fixing the fault. `~/clear_faults` releases the latches but does **not** drop faults that are still being reported -- those re-populate the table and re-trip on the very next report.

**Tier B is level-triggered, not edge-triggered**, on purpose: `should_brake()` is meant to be polled every tick, and `ackermann_mux` is expected to hold the emergency command for as long as `emergency_stop` reads true.

**Disarm-failure fallback:** if `arm_client_->service_is_ready()` is false, or the async response comes back with `success: false`, `disarm_failed_` is set and `emergency_stop` is forced true on every subsequent tick regardless of `brake_level` -- until `~/clear_faults` clears the flag (or a later disarm attempt succeeds). Rationale: a disarm request that silently fails must not leave the vehicle looking "fine" on `/diagnostics`.

**QoS:** the `faults` subscription is `KeepLast(100)` reliable so a burst of reports isn't dropped. Both `emergency_stop` and `autonomy_disarm` publishers are `KeepLast(1)` + `transient_local()` (latched) so a late-joining `ackermann_mux` immediately sees the current state instead of assuming "clear" until the next tick -- this durability setting must match on both ends or the subscription silently never connects.

**Logging:** the disarm request logs `RCLCPP_ERROR` (not throttled -- it only fires once per edge anyway) naming the offending fault(s); the response callback logs `RCLCPP_INFO` on success or `RCLCPP_ERROR` on rejection. `emergency_stop` being asserted logs `RCLCPP_ERROR_THROTTLE` (1000 ms) -- throttled because the condition can persist for a long time (especially latched) and repeated visibility in the log is intentional. Clearing `emergency_stop` logs a single `RCLCPP_INFO`.

## After Launching

1. **Verify diagnostics are publishing with no faults:**

   ```bash
   ros2 topic echo /diagnostics --once
   # Expect one DiagnosticStatus named "fault_collector: summary", level OK, message "No active faults"
   ```

2. **Publish a SEVERE fault (Tier B only) and confirm emergency_stop trips but no disarm is requested:**

   ```bash
   ros2 topic pub /faults wato_fault_msgs/msg/Fault \
     "{level: 2, source: 'test_bench', message: 'severe test fault', id: 'demo_severe', active: true}" --rate 2

   ros2 topic echo /faults/emergency_stop --once   # data: true
   ros2 topic echo /faults/autonomy_disarm --once  # data: false
   ```

3. **Publish a CRITICAL fault (Tier A + B) and confirm the arm service is called once:**

   ```bash
   ros2 topic pub /faults wato_fault_msgs/msg/Fault \
     "{level: 1, source: 'test_bench', message: 'critical test fault', id: 'demo_critical', active: true}" --once

   # Node log should show exactly one "Requesting vehicle disarm via ..." ERROR line, not a repeating one.
   ros2 topic echo /faults/autonomy_disarm --once   # data: true
   ```

4. **Confirm both tiers survive the fault going away (`latch: true` default):**

   ```bash
   # Stop publishing the critical fault and wait > fault_timeout_s (default 5s)
   ros2 topic echo /faults/emergency_stop --once   # still data: true
   ros2 topic echo /faults/autonomy_disarm --once  # still data: true
   ```

5. **Clear the latches:**

   ```bash
   ros2 service call /fault_collector/clear_faults std_srvs/srv/Trigger {}
   ros2 topic echo /faults/emergency_stop --once   # data: false
   ros2 topic echo /faults/autonomy_disarm --once  # data: false
   ```

6. **Confirm the re-engage lockout:** with a CRITICAL fault active and `autonomy_disarm` reading `true`, publish an arm transition and confirm the collector immediately re-disarms:

   ```bash
   ros2 topic pub -1 /oscc_interfacing/is_armed std_msgs/msg/Bool "{data: false}"
   ros2 topic pub -1 /oscc_interfacing/is_armed std_msgs/msg/Bool "{data: true}"
   # expect in the node log:
   #   Requesting vehicle disarm via '/oscc_interfacing/arm': re-arm attempted while a
   #   disarm-level fault is active or latched
   ```

   With no disarm-level fault active, the same arm transition must produce **no** disarm request -- the collector only interferes when a fault warrants it.

## Definition of Good Result

| Check | Expected |
|-------|----------|
| `/diagnostics` publish rate | Matches `publish_rate_hz` |
| No active faults | Summary status level OK, message "No active faults" |
| Active fault present | One summary + one per-fault `DiagnosticStatus`, levels per `diagnostic_level_map` |
| Fault below both tiers | `emergency_stop` and `autonomy_disarm` stay `false` |
| Fault at/below `brake_level` only | `emergency_stop` -> `true`, `autonomy_disarm` stays `false` |
| Fault at/below `disarm_level` | `autonomy_disarm` -> `true`, `arm_service` called exactly once, `emergency_stop` -> `true` |
| `arm_service` unreachable or rejects the call | `emergency_stop` forced `true` regardless of `brake_level`, `RCLCPP_ERROR` logged |
| `latch: true`, fault clears/expires | Tripped tier(s) stay tripped until `~/clear_faults` is called |
| `~/clear_faults` called | Fault table empty, both tiers `false` (assuming no other qualifying faults), disarm-failure flag cleared |
| Operator presses arm button while latched | Vehicle re-arms successfully -- `fault_collector` does not re-disarm in response |
