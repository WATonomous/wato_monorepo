# Ackermann Mux

Priority-based multiplexer for Ackermann drive commands. Handles multiple input sources (e.g., joystick, action planning) and selects the highest priority active input to forward to the vehicle control system.

## Features

- **Priority-based arbitration**: Higher priority inputs take precedence over lower priority ones
- **Input masking**: Enable/disable inputs dynamically via boolean mask topics
- **Safety gating**: Automatically triggers emergency stop if safety-gated inputs stop sending commands within a configurable timeout
- **Emergency fallback**: Publishes a safe emergency command when no eligible inputs are available or safety conditions are tripped

## Configuration

The node is configured via a YAML file (see [config/ackermann_mux_config.yaml](./config/ackermann_mux_config.yaml)). Key parameters:

- `safety_threshold`: Time in seconds before a safety-gated input triggers emergency (default: 0.5s)
- `publish_rate_hz`: Rate at which output commands are published (default: 50.0 Hz)
- `emergency`: Emergency command to publish when no eligible inputs or safety trip occurs
  - `steering_angle`: Emergency steering angle (default: 0.0)
  - `speed`: Emergency speed (default: -0.5)
- `inputs`: List of input sources, each with:
  - `topic`: ROS topic subscribing to `ackermann_msgs/msg/AckermannDriveStamped`
  - `priority`: Priority level (higher number = higher priority)
  - `has_mask`: Whether this input uses a mask topic (default: false)
  - `mask_topic`: Topic publishing `std_msgs/msg/Bool` to enable/disable this input (required if `has_mask: true`)
  - `safety_gating`: Whether to enable safety timeout for this input (default: false)

## Usage

Launch the node with the default configuration:

```bash
ros2 launch ackermann_mux ackermann_mux.launch.yaml
```

The node publishes multiplexed commands to `/ackermann` as `ackermann_msgs/msg/AckermannDriveStamped`.

## How It Works

1. **Safety Check**: If any input with `safety_gating: true` hasn't received a command within `safety_threshold` seconds, the emergency command is published immediately.

2. **Arbitration**: Among all eligible inputs (have received at least one command and are not masked), the one with the highest priority is selected.

3. **Output**: The latest command from the selected input is published with an updated timestamp. If no eligible inputs exist, the emergency command is published.

## Example Configuration

```yaml
ackermann_mux:
  ros__parameters:
    safety_threshold: 0.5
    publish_rate_hz: 50.0
    emergency:
      steering_angle: 0.0
      speed: -0.5
    inputs:
      joystick:
        topic: /joystick/ackermann
        priority: 100 # Highest priority
        has_mask: true
        mask_topic: /joystick/is_idle
        safety_gating: true # Will trigger emergency if joystick stops sending commands
      action:
        topic: /action/ackermann
        priority: 10 # Lower priority than joystick
        has_mask: true
        mask_topic: /action/is_idle
        safety_gating: false
```
