# Eve Joystick

ROS2 node that interfaces with physical joystick hardware and converts joystick input into Ackermann drive commands for vehicle control. Provides safety gating through enable and deadman switches.

## Features

- **Hardware integration**: Uses the ROS `joy` package to read joystick input from USB devices
- **Ackermann command conversion**: Maps joystick axes to steering angle and speed commands
- **Output topic toggling**: Allows switching between standard Ackermann commands and ROSCCO commands via a joystick button
- **Safety gating**: Requires both enable and deadman buttons to be pressed simultaneously for commands to be published
- **Idle state tracking**: Publishes idle status for use by the ackermann mux
- **Configurable mapping**: Supports custom axis assignments, max speeds, steering limits, and axis inversion

## Configuration

The node is configured via a YAML file (see [config/joy_config.yaml](./config/joy_config.yaml)). Key parameters:

### Joystick Node (hardware driver)

- `deadzone`: Joystick deadzone threshold to filter small movements (default: 0.05)

### Joystick Interfacing Node

- `enable_axis`: Joystick axis index for the enable button (must be pressed ≤ -0.9)
- `toggle_button`: Joystick button index to toggle between Ackermann and ROSCCO output (default: 0)
- `steering_axis`: Joystick axis index for steering control (left/right)
- `throttle_axis`: Joystick axis index for throttle control (forward/backward)
- `max_speed`: Maximum speed in m/s (default: 2.0)
- `max_steering_angle`: Maximum steering angle in radians (default: 0.5)
- `invert_steering`: Invert steering direction (default: false)
- `invert_throttle`: Invert throttle direction (default: false)

## Usage

Launch the node with the default configuration:

```bash
ros2 launch joystick_interfacing joystick_interfacing.launch.yaml
```

The node will automatically detect and read from `/dev/input/js0` (or the device specified in the joy_node configuration).

## Topics

### Subscribed

- `/joy` (`sensor_msgs/msg/Joy`): Raw joystick input from the hardware driver

### Published

- `/joystick/ackermann` (`ackermann_msgs/msg/AckermannDriveStamped`): Ackermann drive commands derived from joystick input
- `/joystick/roscco` (`interfacing_custom_msg/msg/Roscco`): ROSCCO drive commands derived from joystick input (used when toggled)
- `/joystick/is_idle` (`std_msgs/msg/Bool`): Idle state indicator (true when joystick is not active)
- `/joystick/state` (`std_msgs/msg/Int8`): Current operation mode (0: NULL, 1: ACKERMANN, 2: ROSCCO)

## How It Works

1. **Hardware Input**: The `joy_node` reads raw joystick data from the USB device and publishes to `/joy`.

2. **Safety Check**: The interfacing node requires the enable axis to be pressed (value ≤ -0.9). If not pressed, a zero-velocity command is published and the idle state is set to `true`.

3. **Command Conversion**: When safety conditions are met:

   - Steering axis is mapped to steering angle: `steering_angle = clamp(axis_value, -1.0, 1.0) * max_steering_angle`
   - Throttle axis is mapped to speed: `speed = clamp(axis_value, -1.0, 1.0) * max_speed`
   - Inversion flags are applied if configured

4. **Topic Selection**: If the toggle button is pressed (rising edge), the node switches between publishing to `/joystick/ackermann` and `/joystick/roscco`.

5. **Output**: The converted command (either Ackermann or ROSCCO) is published with the current timestamp, and the idle state is set to `false`.

## Finding Joystick Axis Indices

To determine which axis indices correspond to your joystick buttons and sticks, you can use:

```bash
ros2 run joy joy_node
ros2 topic echo /joy
```

Then press buttons and move sticks while observing the `axes` and `buttons` arrays in the output. Note that triggers are typically reported as axes, not buttons.
