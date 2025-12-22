# Tracking Evaluation — Developer Guide

## Overview
The `track_eval` package provides on-the-fly evaluation metrics for 2D tracking.
## Dependencies
This package depends on the C++ LAPJV implementation at https://github.com/Vertical-Beach/ByteTrack-cpp.
<br><br>Other dependencies:
- ROS 2: `rclcpp`, `vision_msgs`
- Eigen: `eigen3-cmake-module`, `Eigen3`
## Build
To build, set `ACTIVE_MODULES` in `watod-config.sh` to include `perception` and run

```bash
watod build perception_bringup
```

Alternatively, run

```bash
colcon build --packages-select bytetrack_cpp_vendor track_eval
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

**If launching just the tracker + evaluation:**

```bash
bash -c "ros2 launch tracking_2d tracking_2d_launch.yaml & ros2 launch track_eval track_eval_launch.yaml"
```

## Individual File Purposes
- `Accumulator.{hpp, cpp}`: Defines the Accumulator class that keeps track of all accumulated values needed for metric calculations.
- `Matches.{hpp, cpp}`: Defines the Matches class that holds LAP solution info.
- `cost.{hpp, cpp}`: Contains functions for cost matrix computation.
- `lap.{hpp, cpp}`: Wrapper/pre-processing for ByteTrack-cpp's LAPJV implementation.
- `track_eval.{hpp, cpp}`: The ROS node.

## High Level Flow
1. Receive the ground truth and tracking `Detection2DArray` on the indicated topics
2. Run a linear assignment problem on the ground truths and tracks using `cost = 1 - IoU`
3. Update a metric Accumulator with the resulting Matches
4. Calculate the required tracking metrics (e.g. MOTP, MOTA, IDF1, etc.)
