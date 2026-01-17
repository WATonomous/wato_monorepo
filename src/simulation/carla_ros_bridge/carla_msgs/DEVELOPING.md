# Developing carla_msgs

## Architecture

Standard ROS 2 message package using CMake build system with `rosidl_generate_interfaces`.

## Adding New Messages

1. Create `.msg` file in `msg/` directory
2. Add to `CMakeLists.txt`:
   ```cmake
   rosidl_generate_interfaces(${PROJECT_NAME}
     "msg/MyMessage.msg"
     ...
   )
   ```
3. Rebuild package

## Adding New Services

1. Create `.srv` file in `srv/` directory
2. Add to `CMakeLists.txt`
3. Rebuild package

## Message Format

```
# Comment describing the field
type field_name
```

## Service Format

```
# Request
type request_field
---
# Response
type response_field
```

## Dependencies

If messages reference other packages' types, add to:
- `package.xml`: `<depend>other_msgs</depend>`
- `CMakeLists.txt`: `DEPENDENCIES other_msgs`
