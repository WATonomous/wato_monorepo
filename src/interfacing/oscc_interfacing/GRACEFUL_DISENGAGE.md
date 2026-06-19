# Graceful Disengagement (Steering Torque Handover)

## Problem

When autonomy disengaged, `oscc_interfacing` called `oscc_disable()` immediately on
every disengage path. The OSCC steering board stops applying torque the instant it
is disabled, so the commanded steering torque dropped from whatever it was (often
large, plus the deadzone offset) to zero in a single CAN cycle. The result was a
**hard step in steering torque that jerked the wheel**, and the actuator did **not
hold any torque** during the transition — control snapped back to the operator with
no managed handover.

This is unsafe for an operator-in-the-loop handover: the wheel can kick, and on a
crowned or curved road the wheel can dart toward center the moment authority is cut.

## What changed

A **graceful disengage** was added: on an *intentional* disarm, the node ramps the
steering torque command linearly from its current value down to exactly zero over a
configurable time, and only then disables the boards. This removes the step and
gives the operator a smooth, predictable handover.

Crucially, this applies **only to the planned/manual disarm path**. The
safety-reactive paths still disable instantly (see the table below).

### Disengage paths and their behavior

| Path | Source | Behavior | Why |
|------|--------|----------|-----|
| Manual disarm | `arm_service_callback` (joystick button / service call `data:false`) | **Ramp** steering torque → 0, then disable | Intentional handover; smoothness matters |
| Operator override | `process_events` (board reports `operator_override`) | **Instant** disable, aborts any ramp | Driver is physically fighting the actuator — never ramp against them |
| Hardware fault | `process_events` (fault report, if `disable_boards_on_fault`) | **Instant** disable, aborts any ramp | State is untrusted; never hold torque through a fault |
| OSCC API error | `handle_any_errors` | **Instant** disable, aborts any ramp | Comms/state untrusted |

## How it works

### State machine

A three-state lifecycle (`DisengageState`) is tracked, all mutated only inside the
serialized "Group A" callbacks:

```
ENGAGED ──manual disarm──► DISENGAGING ──ramp done──► DISABLED
   ▲                            │
   └──arm success               └──override / fault / error / arm─► DISABLED
```

- **ENGAGED** — normal operation; autonomy commands from `roscco` are applied.
- **DISENGAGING** — rampdown in progress; autonomy commands are **ignored** so the
  planner cannot re-inject torque mid-handover.
- **DISABLED** — boards disabled, operator has full control.

### Non-blocking ramp

The rampdown is **not** a blocking loop — that would freeze the executor and stop
the node from processing CAN reports (override/fault) during the handover. Instead
it is advanced one step per tick of the existing 5 ms `event_timer_` (the same timer
that already polls override/fault), via `tick_disengage()`:

1. `begin_disengage()` captures the currently-applied torque and the start time,
   cuts throttle to zero immediately, and sets state to `DISENGAGING`. The service
   responds right away with "disengage initiated".
2. Each `process_events()` tick computes
   `torque = initial_torque * (1 - elapsed/duration)` and publishes it.
3. When `elapsed >= duration`, it publishes exactly `0.0`, disables the configured
   modules, and transitions to `DISABLED`.

`tick_disengage()` runs **after** the override/fault checks in `process_events()`,
so a genuine abort always wins over the ramp.

### Deadzone is bled off with the ramp

The applied steering torque includes the deadzone compensation offset
(`steering_torque_deadzone_pos/neg`, ~0.09–0.13). The ramp starts from the **final**
applied torque (`last_steering_torque_cmd_`, captured in `roscco_callback` after the
deadzone is added) and ramps that whole value to zero. This means:

- **No initial step** — the ramp begins from exactly what is currently applied.
- **No residual end step** — the deadzone offset is bled off as part of the ramp,
  rather than being held until the boards cut out.

### Throttle and brake during handover

Throttle is set to zero immediately in `begin_disengage()` — we must not keep
accelerating while handing over. Braking authority is left untouched during the
ramp (so the vehicle is not destabilised) and is released together with the boards
when the ramp completes.

## Safety properties

- **Bounded duration.** The ramp is purely time-based, so it always completes within
  `disarm_ramp_ms`; autonomy torque is never held indefinitely.
- **Abort always available.** Because the ramp is timer-driven and non-blocking,
  override/fault/error detection stays live throughout and pre-empts the ramp.
- **No command injection during handover.** `roscco_callback` ignores commands
  unless state is `ENGAGED`.
- **Fail-safe config.** `disarm_ramp_ms <= 0` disables graceful disarm (falls back to
  instant disable). `enable_graceful_disarm: false` does the same explicitly.
- **Reactive paths unchanged.** Override, fault, and error handling remain immediate.

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `enable_graceful_disarm` | bool | `true` | Ramp steering torque to zero on manual disarm before disabling boards |
| `disarm_ramp_ms` | double | `600.0` | Duration (ms) of the steering torque rampdown |

## Files changed

| File | Change |
|------|--------|
| `include/oscc_interfacing/oscc_interfacing.hpp` | `DisengageState` enum; `begin_disengage()`, `tick_disengage()`, `disable_modules()` declarations; ramp state + parameter members; `last_steering_torque_cmd_` |
| `src/oscc_interfacing.cpp` | Param declare/read; record final torque in `roscco_callback`; gate commands on state; graceful path + helpers; ramp tick + abort resets in `process_events`/`handle_any_errors` |
| `config/oscc_interfacing_config.yaml` | New parameters |
| `DEVELOPING.md` | Parameter table + link to this doc |

## Limitations / future work (Phase 2)

This is the deterministic feedforward backbone. It removes the jerk but does **not**
actively control the vehicle's path during the ramp. Possible follow-ups:

- **Angle-hold during ramp.** Use the measured steering angle
  (`latest_steering_angle_`) as a PID setpoint and scale the PID output by the ramp
  fraction, so the vehicle holds its heading while authority bleeds off (prevents
  drift on crowned/curved roads). Requires conservative, rate-limited gains so the
  controller does not itself introduce oscillation.
- **Driver-torque arbitration.** `steering_report.torque` (steering_can_protocol.h)
  is a *measured* sensor value, distinct from the command. It could be used to
  shorten the ramp as the operator's hands-on torque is detected, completing the
  handover as soon as the driver has the wheel — with the time-based ramp as the
  backstop.
