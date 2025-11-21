# 2D Tracking Visualization — Developer Guide
This document provides technical specifications and details for the visualization of the ByteTrack ROS 2 wrapper in this repository. It explains directory layout, dependencies, build details usage details and what the code does at a high level.
## Overview
The `track_viz_2d` package provides visualization for the tracker node. It publishes annotation boxes that visualize both the received detections and the output tracks.
## Package Organization

```
track_viz_2d
|
|── config
|   └── params.yaml
|
|── docs
|   └── DEVELOPING.md
|
|── include
|   └── track_viz_2d.hpp
|
|── launch
|   └── track_viz_2d_launch.yaml
|
|── src
|   └── track_viz_2d.cpp
|
|── CMakeLists.txt
└── package.xml
```

## Dependencies
- ROS 2: `foxglove_msgs`, `std_msgs`, `vision_msgs`
## Build
To build, set `ACTIVE_MODULES` in `watod-config.sh` to include `perception` and run

```bash
watod build perception_bringup
```

Alternatively, you can also run

```bash
colcon build --packages-select track_viz_2d
```

to debug without creating a container.
## Usage
To start the container, run

```bash
watod up perception_bringup
```

with the command in `docker-compose.perception.yaml` set to

```bash
bash -c "ros2 launch tracking_2d tracking_2d_launch.yaml & ros2 launch track_viz_2d track_viz_2d_launch.yaml"
```

## High Level Flow
1. Receive the detection and tracking `Detection2DArray` messages on the indicated topics
2. Fill an `ImageAnnotations` message with `PointsAnnotation` messages created using the coordinates and size of each `Detection2D`
3. Publish the fully populated `ImageAnnotations` message
