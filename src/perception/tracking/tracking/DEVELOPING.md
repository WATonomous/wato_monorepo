# Multi-Object Tracking — Developer Guide

## Dependencies

Refer to [`package.xml`](./package.xml)

## Building

**Deployment**

```bash
watod build perception_bringup
```

**Development**

```bash
colcon build --packages-select bytetrack_cpp_vendor tracking wato_lifecycle_manager
```

## Unit Testing

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

ByteTrack currently uses a modified fork of [`ByteTrack-cpp`](https://github.com/Vertical-Beach/ByteTrack-cpp) located at [`ByteTrack-cpp-3d`](https://github.com/khuang9/ByteTrack-cpp-3d). Integration with ROS2 nodes happens through the vendor package at [`bytetrack_cpp_vendor/`](../bytetrack_cpp_vendor/).

See [`bytetrack_cpp_vendor/DEVELOPING.md`](../bytetrack_cpp_vendor/DEVELOPING.md) for more details.
