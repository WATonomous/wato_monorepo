# Mock Action Publisher

Mock node for testing and development that publishes simulated Ackermann drive commands on behalf of the action planning system. Provides a service interface to toggle idle state, allowing developers to test the ackermann mux and vehicle control without requiring the full action planning stack.

## Features

- **Mock command generation**: Publishes configurable Ackermann drive commands at a specified rate
- **Idle state control**: Service-based interface to toggle idle state on/off
- **Idle state publishing**: Continuously publishes idle status for use by the ackermann mux
- **Zero-velocity when idle**: Automatically publishes zero commands when in idle state

## Configuration

The node is configured via a YAML file (see [config/mock_action_publisher.yaml](./config/mock_action_publisher.yaml)). Key parameters:

- `publish_rate_hz`: Rate at which Ackermann commands are published (default: 20.0 Hz)
- `speed`: Speed command to publish when not idle, in m/s (default: 1.0)
- `steering_angle`: Steering angle command to publish when not idle, in radians (default: 1.0)

## Usage

Launch the node with the default configuration:

```bash
ros2 launch mock_action_publisher mock_action_publisher.launch.yaml
```

## Topics

### Published

- `/action/ackermann` (`ackermann_msgs/msg/AckermannDriveStamped`): Mock Ackermann drive commands
  - When not idle: publishes configured `speed` and `steering_angle`
  - When idle: publishes zero speed and zero steering angle
- `/action/is_idle` (`std_msgs/msg/Bool`): Idle state indicator (published at 10 Hz, used by ackermann mux for masking)

## Services

### `/action/set_idle` (`std_srvs/srv/SetBool`)

Service to toggle the idle state of the mock action publisher.

**Example usage:**

```bash
# Set to idle (publishes zero commands)
ros2 service call /action/set_idle std_srvs/srv/SetBool "{data: true}"

# Activate (publishes configured commands)
ros2 service call /action/set_idle std_srvs/srv/SetBool "{data: false}"
```

## How It Works

1. **Command Publishing**: At the configured `publish_rate_hz`, the node publishes Ackermann commands:

   - If `is_idle == false`: Publishes commands with configured `speed` and `steering_angle`
   - If `is_idle == true`: Publishes zero speed and zero steering angle

2. **Idle State Publishing**: At 10 Hz (independent of command rate), the node publishes the current idle state to `/action/is_idle`. This ensures the ackermann mux always has a fresh mask signal.

3. **Service Control**: The `/action/set_idle` service allows external nodes or users to toggle the idle state, which immediately affects the commands being published.

## Example Configuration

```yaml
mock_action_publisher:
  ros__parameters:
    publish_rate_hz: 20.0
    speed: 1.0
    steering_angle: 1.0
```
