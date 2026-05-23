# oscc_interfacing

Hardware interface to the OSCC (Open Source Car Control) boards via CAN. Sends steering torque, throttle, and brake commands and reads back wheel speeds, steering feedback, and fault state.

## Overview

OSCC is the low-level by-wire system on Eve. This node wraps the OSCC C library, which manages direct CAN communication with the steering, throttle, and brake modules. The node exposes a clean ROS interface: subscribe to a ROSCCO command, provide an arm/disarm service, and publish feedback.

## Architecture

```
[oscc_mux] ──► /roscco ──► oscc_interfacing_node ──► OSCC C library ──► CAN bus ──► vehicle
                                     │
                                     ├──► /oscc_interfacing/is_armed
                                     ├──► /oscc_interfacing/wheel_speeds
                                     └──► /oscc_interfacing/steering_angle
```

**Arming** enables the OSCC modules (steering, throttle, brakes) so they accept commands. The vehicle must be armed before any actuation occurs. The joystick operator arms via button press, which calls the `/oscc_interfacing/arm` service.

**Fault handling:** The OSCC library fires callbacks when faults are detected (e.g., operator brake/throttle override). The node detects these and either disarms or disables individual modules depending on `disable_boards_on_fault`. Override detection (driver touching the wheel or pedals) automatically disarms.

The OSCC library manages CAN communication in its own background context. The ROS node caches the latest feedback values and publishes them at a fixed rate.
