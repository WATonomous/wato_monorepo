# Eve Joystick

ROS2 node that interfaces with physical joystick hardware and converts joystick input into Ackermann drive commands for vehicle control. Provides safety gating through enable and deadman switches.

## Features

- **Hardware integration**: Uses the ROS `joy` package to read joystick input from USB devices
- **Ackermann command conversion**: Maps joystick axes to steering angle and speed commands
- **OSCC Arming Control**: Interface to arm/disarm the OSCC board via a service call with vibration feedback
- **Output topic toggling**: Allows switching between standard Ackermann commands and ROSCCO commands via a joystick button
- **Safety gating**: Requires the enable button to be held for commands to be published
- **Idle state tracking**: Publishes idle status for use by the ackermann mux
- **Configurable mapping**: Supports custom axis assignments, max speeds, steering limits, and axis inversion

## Configuration

The node is configured via a YAML file (see [config/joy_config.yaml](./config/joy_config.yaml)). Key parameters:

### Joystick Node (hardware driver)

- `deadzone`: Joystick deadzone threshold to filter small movements (default: 0.05)

### Joystick Interfacing Node

- `enable_axis`: Joystick axis index for the enable button (must be pressed ≤ -0.9)
- `toggle_button`: Joystick button index to toggle between Ackermann and ROSCCO output (default: 0)
- `arming_button`: Joystick button index to arm/disarm the vehicle (default: 0)
- `steering_axis`: Joystick axis index for steering control (left/right)
- `throttle_axis`: Joystick axis index for throttle control (forward/backward)
- `ackermann_max_speed`: Maximum speed in m/s for Ackermann mode (default: 2.0)
- `ackermann_max_steering_angle`: Maximum steering angle in radians for Ackermann mode (default: 0.5)
- `roscco_max_speed`: Maximum speed in m/s for ROSCCO mode (default: 1.0)
- `roscco_max_steering_angle`: Maximum steering angle in radians for ROSCCO mode (default: 0.3)
- `invert_steering`: Invert steering direction (default: false)
- `invert_throttle`: Invert throttle direction (default: false)
- `toggle_vibration_intensity`: Rumble intensity during toggles (default: 0.5)
- `toggle_vibration_duration_ms`: Duration of rumble pulses in ms (default: 100)

## Usage

Launch the node with the default configuration:

```bash
ros2 launch joystick_interfacing joystick_interfacing.launch.yaml
```

The node will automatically detect and read from `/dev/input/js0` (or the device specified in the joy_node configuration).

## Topics and Services

### Subscribed

- `/joy` (`sensor_msgs/msg/Joy`): Raw joystick input from the hardware driver
- `/oscc_interfacing/is_armed` (`std_msgs/msg/Bool`): Current arming state of the vehicle

### Published

- `/joystick/ackermann` (`ackermann_msgs/msg/AckermannDriveStamped`): Ackermann drive commands
- `/roscco` (`roscco_msg/msg/Roscco`): ROSCCO drive commands (used when toggled)
- `/joystick/is_idle` (`std_msgs/msg/Bool`): Idle state indicator (true when joystick is not active)
- `/joystick/state` (`std_msgs/msg/Int8`): Current operation mode (0: NULL, 1: ACKERMANN, 2: ROSCCO)
- `/joy/set_feedback` (`sensor_msgs/msg/JoyFeedback`): Force feedback (vibration) commands to the joystick

### Service Clients

- `/oscc_interfacing/arm` (`std_srvs/srv/SetBool`): Service to arm (true) or disarm (false) the vehicle

## How It Works

1. **Hardware Input**: The `joy_node` reads raw joystick data from the USB device and publishes to `/joy`.

2. **Arming Control**: When the `arming_button` is pressed (rising edge), the node calls the `/oscc_interfacing/arm` service to toggle the vehicle state.
   - **Arming**: Triggers a double vibration pulse.
   - **Disarming**: Triggers a single long (500ms) vibration pulse.

3. **Safety Check**: The interfacing node requires the enable axis to be pressed (value ≤ -0.9). If not pressed, a zero-velocity command is published and the idle state is set to `true`.

4. **Command Conversion**: When safety conditions are met:
   - Steering and throttle are scaled based on the current mode (Ackermann or ROSCCO) using their respective `max_speed` and `max_steering_angle` parameters.
   - Inversion flags are applied if configured.

5. **Topic Selection**: If the `toggle_button` is pressed (rising edge), the node switches between publishing to `/joystick/ackermann` and `/roscco`.
   - **Switch to ROSCCO**: Double vibration pulse.
   - **Switch to Ackermann**: Single vibration pulse.

6. **Output**: The converted command is published with the current timestamp, and the idle state is set to `false`.

## Feedback Patterns

The node provides haptic feedback via joystick vibration:
- **Toggle to ROSCCO**: 2 pulses (default duration)
- **Toggle to Ackermann**: 1 pulse (default duration)
- **Armed Successfully**: 2 pulses (default duration)
- **Disarmed Successfully**: 1 long pulse (500ms)

## Finding Joystick Axis Indices

To determine which axis indices correspond to your joystick buttons and sticks, run the joy_node and echo the /joy topic. Then press buttons and move sticks while observing the `axes` and `buttons` arrays in the output. Note that triggers are typically reported as axes, not buttons.
