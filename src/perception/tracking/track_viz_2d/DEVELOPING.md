# 2D Tracking Visualization — Developer Guide

## Overview
The `track_viz_2d` package provides visualization for the tracker node. It publishes annotation boxes that visualize both the received detections and the output tracks.
## Dependencies
- ROS 2: `foxglove_msgs`, `std_msgs`, `vision_msgs`
## Build
To build, set `ACTIVE_MODULES` in `watod-config.sh` to include `perception` and run

```bash
watod build perception_bringup
```

Alternatively, run

```bash
colcon build --packages-select track_viz_2d
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

**If launching just the tracker + visualization:**

```bash
bash -c "ros2 launch tracking_2d tracking_2d_launch.yaml & ros2 launch track_viz_2d track_viz_2d_launch.yaml"
```

## High Level Flow
1. Receive the detection and tracking `Detection2DArray` messages on the indicated topics
2. Fill an `ImageAnnotations` message with `PointsAnnotation` messages created using the coordinates and size of each `Detection2D`
3. Publish the fully populated `ImageAnnotations` message
