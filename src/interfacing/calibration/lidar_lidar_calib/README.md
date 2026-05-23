# lidar_lidar_calib

LiDAR-to-LiDAR extrinsic calibration using GICP (Generalized ICP) point cloud registration. Computes the rigid transform between the center LiDAR and each side LiDAR (NE, NW).

## Overview

The NE and NW LiDARs are side-mounted and their transforms relative to the center LiDAR must be known for `lidar_aggregator` to merge the point clouds correctly. This package takes simultaneous scans from two LiDARs and finds the transform that best aligns them.

GICP is an iterative refinement algorithm — it requires a reasonable initial guess (from physical measurement) to converge. The calibration must be run with the vehicle stationary in a feature-rich environment.

## Prerequisites

- Vehicle must be stationary
- Environment must have sufficient geometric features (not an empty, symmetric room)
- A physical measurement of the approximate LiDAR mount positions must be available as the initial guess

## Usage

```bash
colcon build --packages-select lidar_lidar_calib
source install/setup.bash
ros2 launch lidar_lidar_calib lidar_lidar_calib.yaml
```

This launches two calibration instances in parallel — one for NE and one for NW — each registering its side LiDAR against the center LiDAR. The resulting transforms are logged and published.

Once calibration converges consistently, update the corresponding `<origin>` blocks in `eve_description/urdf/` with the calibrated values.
