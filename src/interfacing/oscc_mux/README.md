# oscc_mux

Priority-based multiplexer for ROSCCO commands (`roscco_msg/Roscco`). Same architecture as `ackermann_mux` but operates on the direct torque/throttle interface rather than Ackermann commands.

## Overview

When the vehicle is in ROSCCO mode, multiple sources may want to send steering torque and throttle/brake commands. `oscc_mux` arbitrates between them using configurable priority levels and feeds the selected command to `oscc_interfacing`.

## Architecture

```
/joystick/roscco  (priority 100) ─┐
/pid/roscco       (priority  10) ─┤──► oscc_mux ──► /roscco ──► oscc_interfacing
```

The joystick has higher priority so operator intervention always overrides autonomous commands. Unlike `ackermann_mux`, safety gating is typically disabled here — the joystick may stop publishing ROSCCO when in Ackermann mode, and that should not trigger an emergency. Safety is handled at the `ackermann_mux` level instead.

See [ackermann_mux](../ackermann_mux/README.md) for the full description of priority arbitration, masking, and safety gating logic — `oscc_mux` is an identical implementation for the ROSCCO message type.
