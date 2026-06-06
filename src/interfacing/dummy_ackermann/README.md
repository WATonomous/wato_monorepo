# dummy_ackermann

Synthetic Ackermann command generators for bench testing and bringup without a physical joystick or autonomous planner.

## Overview

Two nodes generate deterministic Ackermann command profiles that can be injected into `ackermann_mux` at low priority for hardware-in-the-loop testing:

- **`ackermann_square_wave_node`** — oscillates steering angle between `+amplitude` and `-amplitude` at a configurable period, with zero speed.
- **`ackermann_velocity_trapezoid_node`** — ramps speed through a trapezoid profile (rise → hold → ramp down → hold at zero), with zero steering.

Neither node has subscribers or feedback. They publish unconditionally at a fixed rate once activated.
