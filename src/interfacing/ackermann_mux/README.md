# ackermann_mux

Priority-based multiplexer for Ackermann drive commands (`ackermann_msgs/AckermannDriveStamped`). Sits between all Ackermann command sources (joystick, action planner, test nodes) and the PID controller, selecting the highest-priority active input at each publish cycle.

## Overview

Multiple subsystems may simultaneously want to control the vehicle — a joystick operator, an autonomous planner, or a test signal generator. `ackermann_mux` arbitrates between them using configurable priority levels, so the joystick always overrides the planner, which overrides test nodes, without any source needing to know about the others.

## Architecture

```
/joystick/ackermann  (priority 200) ─┐
/action/ackermann    (priority 100) ─┤─► ackermann_mux ─► /ackermann ─► pid_control
/dummy/ackermann     (priority  10) ─┘
```

On each publish tick the node:
1. Checks whether any safety-gated input has gone stale — if so, publishes the emergency command immediately.
2. Iterates inputs from highest to lowest priority, skipping masked-out inputs.
3. Publishes the latest command from the first eligible input, or the emergency command if none qualify.

**Safety gating** monitors command age on critical inputs (typically the joystick). If the joystick stops sending within `safety_threshold` seconds, the mux publishes the emergency command (brake) rather than holding the last known command.

**Input masking** lets a source signal that it is voluntarily idle (e.g., `joystick/ackermann_is_idle` when the enable trigger is released) so lower-priority inputs can take over without triggering a safety trip.
