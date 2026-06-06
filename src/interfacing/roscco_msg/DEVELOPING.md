# Developing roscco_msg

## Build

```bash
colcon build --packages-select roscco_msg
```

## Adding a New Message

1. Create `msg/MyMessage.msg` following the [ROS 2 message format](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html).
2. Add it to `CMakeLists.txt` under `rosidl_generate_interfaces`.
3. Rebuild and update any packages that use the new message type.

## Field Naming Conventions

Wheel-related fields use compass directions: `ne` (right-front), `nw` (left-front), `se` (right-rear), `sw` (left-rear). N = front, S = rear, E = right, W = left. This matches the convention used in `can_state_estimator` and `oscc_interfacing`.
