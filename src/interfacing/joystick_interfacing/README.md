# joystick_interfacing

Converts raw joystick input into Ackermann or ROSCCO drive commands with safety gating, arming control, and haptic feedback.

## Overview

The `joystick_node` wraps `joy_node` (the generic ROS joystick driver) and adds vehicle-specific logic: a physical enable trigger for safety, mode toggling between Ackermann and ROSCCO control, and OSCC arm/disarm via joystick button. It feeds directly into `ackermann_mux` and `oscc_mux`.

## Architecture

```
USB joystick
     │
[joy_node] → /joy
     │
[joystick_node] ──► /joystick/ackermann         → ackermann_mux (priority 100)
                ──► /joystick/roscco             → oscc_mux      (priority 100)
                ──► /joystick/ackermann_is_idle  → ackermann_mux masking
                ──► /joystick/roscco_is_idle     → oscc_mux masking
                ──► /joy/set_feedback            → haptic rumble
                ──► /oscc_interfacing/arm        (service call)
```

## Modes

The node operates in one of two modes at a time, toggled via `toggle_button`:

- **ACKERMANN** — publishes to `/joystick/ackermann` with speed and steering angle limits
- **ROSCCO** — publishes to `/joystick/roscco` with separate torque and speed limits

The enable axis must be held (value ≤ −0.9) for commands to be non-zero. When released, the node publishes zero commands and sets both idle flags to `true`, allowing lower-priority inputs in the mux to take over.

Mode switching is **blocked while the deadman is held** — release the enable trigger before pressing `toggle_button`. This prevents accidental pipeline switches mid-drive.

## Haptic Feedback Patterns

| Event | Pattern |
|-------|---------|
| Toggle to ROSCCO | 2 pulses |
| Toggle to Ackermann | 1 pulse |
| Armed successfully | 2 pulses |
| Disarmed successfully | 1 long pulse (500 ms) |
