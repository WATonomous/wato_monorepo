# interfacing_bringup

Meta-package that coordinates launch of all interfacing subsystems for Eve — sensors, CAN/control, and health monitoring.

## Overview

Rather than launching drivers and control nodes individually, this package provides a single entry point. It composes three subsystem launch files and a shared config file so the full stack comes up with one command.

## Architecture

```
interfacing.launch.yaml
├── interfacing_sensors.launch.yaml     GPS, LiDARs, cameras
├── interfacing_can.launch.yaml         joystick, mux, PID, OSCC, CAN state estimator
└── topic_healthchecker                 liveness monitoring over HTTP
```

**Sensor drivers** are third-party ROS packages declared as rosdep keys and launched directly:
- Novatel OEM7 (GPS + IMU)
- FLIR Blackfly GigE (cameras, 12 total)
- Velodyne VLP32C / VLP16 (LiDARs, 3 total)

**CAN/control stack** is all first-party code in this repo:
`joy_node` → `joystick_node` → `ackermann_mux` → `ackermann_smoother` → `pid_control` → `oscc_mux` → `oscc_interfacing`

All parameters for every subsystem are consolidated in `config/interfacing.yaml`.
