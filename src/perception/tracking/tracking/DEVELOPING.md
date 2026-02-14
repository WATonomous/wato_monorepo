# Multi-Object Tracking — Developer Guide

## Dependencies

Refer to package.xml

## Building

**Deployment**

```bash
watod build perception_bringup
```

**Development**

```bash
colcon build --packages-select bytetrack_cpp_vendor tracking
```

## Testing

```bash
colcon build --packages-select bytetrack_cpp_vendor tracking wato_test
colcon test --packages-select tracking
colcon test-result --all --verbose
```

## Launching

```bash
source install/setup.bash
ros2 launch tracking tracking_launch.yaml
```

## Modifying ByteTrack
ByteTrack modifications are currently implemented as patch files for the ByteTrack vendor package. Patch files can be found under `tracking/bytetrack_cpp_vendor/patches`.
All patches are implemented for [ByteTrack-cpp](https://github.com/Vertical-Beach/ByteTrack-cpp).
