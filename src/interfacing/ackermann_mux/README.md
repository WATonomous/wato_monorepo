# ackermann_mux

Priority-based multiplexer for Ackermann drive commands (`ackermann_msgs/AckermannDriveStamped`). Sits between all Ackermann command sources (joystick, action planner, test nodes) and the PID controller, selecting the highest-priority active input at each publish cycle.

## Overview

Multiple subsystems may simultaneously want to control the vehicle — a joystick operator, an autonomous planner, or a test signal generator. `ackermann_mux` arbitrates between them using configurable priority levels, so the joystick always overrides the planner, which overrides test nodes, without any source needing to know about the others.

Optionally, `ackermann_mux` can also subscribe to `fault_collector`'s latched `emergency_stop` signal (see `fault_estop.enabled`). This is a hard override that outranks both priority arbitration and safety gating: while it is asserted, the mux publishes the emergency command — **except** for inputs listed in `fault_estop.bypass_inputs` (the joystick, by default), which remain selectable so the safety driver is never locked out of the vehicle. This is `fault_collector`'s Tier B (brake) response; Tier A (disarm) is a separate, one-shot service call `fault_collector` makes directly to `oscc_interfacing` and does not go through `ackermann_mux` at all.

## Architecture

```
/joystick/ackermann  (priority 200) ─┐
/action/ackermann    (priority 100) ─┤─► ackermann_mux ─► /ackermann ─► pid_control
/dummy/ackermann     (priority  10) ─┘         ▲
                                                │
        /faults/emergency_stop ─────────────────┘  (hard override, except bypass_inputs e.g. joystick)
```

On each publish tick the node:
1. Checks whether the fault emergency stop is active (if `fault_estop.enabled`) — if so, publishes the highest-priority eligible bypass input's command if one exists (operator retains control), or the emergency command otherwise, and skips everything below.
2. Checks whether any safety-gated input has gone stale — if so, publishes the emergency command immediately.
3. Iterates inputs from highest to lowest priority, skipping masked-out inputs.
4. Publishes the latest command from the first eligible input, or the emergency command if none qualify.

**Safety gating** monitors command age on critical inputs (typically the joystick). If the joystick stops sending within `safety_threshold` seconds, the mux publishes the emergency command (brake) rather than holding the last known command.

**Input masking** lets a source signal that it is voluntarily idle (e.g., `joystick/ackermann_is_idle` when the enable trigger is released) so lower-priority inputs can take over without triggering a safety trip.
