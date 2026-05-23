# pid_control

Low-level vehicle control nodes that close the loop between Ackermann setpoints and ROSCCO actuator commands. Three lifecycle nodes cover different stages of the control pipeline.

## Overview

The control chain converts planner outputs (desired steering angle + speed) into hardware torque and throttle commands for the OSCC system:

```
[planner] ──► ackermann_smoother ──► vel_driven_feedforward_pid ──► [oscc_mux]
                                           ▲              ▲
                              steering_feedback     velocity_feedback
```

## Nodes

### ackermann_smoother_node

Rate-limiter between the planner and the PID controller. Prevents abrupt setpoint jumps by clamping the rate of change of steering angle and speed. Produces smoother hardware commands and reduces mechanical stress.

### pid_control_node

Basic dual PID controller. Runs independent PID loops for steering (setpoint = desired angle, feedback = current angle) and velocity (setpoint = desired speed, feedback = current speed), and combines their outputs into a single ROSCCO command.

### vel_driven_feedforward_pid_node

Extended controller with a velocity-dependent feedforward term on top of steering PID. Improves tracking at speed by pre-compensating for the known steering torque required at a given velocity.

**Feedforward model:**
```
T_ff = (c0 + c1·v + c2·v²) · steering_setpoint + friction_offset · sign(steering_setpoint)
```

Polynomial coefficients are fit from bag data using the scripts in `analysis/`. D-on-measurement avoids derivative kick when the setpoint changes. An EMA filter smooths noisy velocity feedback.
