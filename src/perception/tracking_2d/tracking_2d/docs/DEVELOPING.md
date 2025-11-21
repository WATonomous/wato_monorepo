# ByteTrack 2D Tracking — Developer Guide
This document provides technical specifications and details for the ByteTrack ROS 2 wrapper in this repository. It explains directory layout, dependencies, build details usage details and what the code does at a high level.
## Overview
The `tracking_2d` package serves as a ROS 2 wrapper for ByteTrack. It associates and tracks 2D detection bounding boxes in the image frame.
## Package Organization

```
tracking_2d
|
|── config
|   └── params.yaml
|
|── docs
|   └── DEVELOPING.md
|
|── include
|   └── tracking_2d.hpp
|
|── launch
|   └── tracking_2d_launch.yaml
|
|── src
|   └── tracking_2d.cpp
|
|── THIRD_PARTY_LICENSES
|   └── BYTETRACK_LICENSE
|
|── CMakeLists.txt
└── package.xml
```

## Dependencies
This package depends on the C++ ByteTrack implementation at https://github.com/Vertical-Beach/ByteTrack-cpp.
<br><br>Other dependencies:
- ROS 2: `rclcpp`, `std_msgs`, `vision_msgs`
- Eigen: `eigen3-cmake-module`, `Eigen3`
## Build
To build, set `ACTIVE_MODULES` in `watod-config.sh` to include `perception` and run

```bash
watod build perception_bringup
```

Alternatively, you can also run

```bash
colcon build --packages-select tracking_2d --cmake-args -DBYTETRACK_DIR=/path/to/your/bytetrack-repo
```

if using your own ByteTrack build, or if you just want to debug without creating a container.
## Usage
To start the container, run

```bash
watod up perception_bringup
```

with the command in `docker-compose.perception.yaml` set to

```bash
bash -c "ros2 launch tracking_2d tracking_2d_launch.yaml
```

or

```bash
bash -c "ros2 launch tracking_2d tracking_2d_launch.yaml & ros2 launch track_viz_2d track_viz_2d_launch.yaml"
```

if running visualization.

## High Level Flow
1. Receive a `Detection2DArray` on the indicated detection topic
2. Convert the `Detection2DArray` to an `std::vector<byte_track::Object>`
3. Pass the objects to ByteTrack to update the tracks with the new detections
4. Convert the updated tracks from `std::vector<byte_track::BYTETracker::STrackPtr>` back to `Detection2DArray` and publish
