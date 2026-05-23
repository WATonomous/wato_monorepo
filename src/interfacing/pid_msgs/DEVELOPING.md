# Developing pid_msgs

## Build

```bash
colcon build --packages-select pid_msgs
```

## Adding a New Message

1. Create `msg/MyMessage.msg` following the [ROS 2 message format](https://docs.ros.org/en/humble/Concepts/About-ROS-Interfaces.html).
2. Add it to `CMakeLists.txt` under `rosidl_generate_interfaces`.
3. Rebuild and update any packages that use the new message type.
