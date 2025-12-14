# ByteTrack 2D Tracking — Developer Guide

## Overview
The `tracking_2d` package serves as a ROS 2 wrapper for ByteTrack. It associates and tracks 2D detection bounding boxes in the image frame.
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

Alternatively, run

```bash
colcon build --packages-select bytetrack_cpp_vendor tracking_2d
```

to debug without creating a container.
## Usage
To start the container, run

```bash
watod up perception_bringup
```

The command in `docker-compose.perception.yaml` should be set to one of the following:

**If launching the entire perception stack:**

```bash
bash -c "ros2 launch perception_bringup perception_bringup_launch.yaml"
```

**If launching just the tracker:**

```bash
bash -c "ros2 launch tracking_2d tracking_2d_launch.yaml"
```

**If launching the tracker with visualization:**

```bash
bash -c "ros2 launch tracking_2d tracking_2d_launch.yaml & ros2 launch track_viz_2d track_viz_2d_launch.yaml"
```

## High Level Flow
1. Receive a `Detection2DArray` on the indicated detection topic
2. Convert the `Detection2DArray` to an `std::vector<byte_track::Object>`
3. Pass the objects to ByteTrack to update the tracks with the new detections
4. Convert the updated tracks from `std::vector<byte_track::BYTETracker::STrackPtr>` back to `Detection2DArray` and publish
