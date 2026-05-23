# lidar_imu_calib

Online LiDAR–IMU extrinsic calibration using the RILI (Robust real-time LiDAR–Inertial initialization) algorithm.

## Overview

Before fusing LiDAR point clouds with IMU data for localization (e.g., in `eidos`), the rigid body transform between the LiDAR frame and the IMU frame must be known. This package estimates that transform online — without a calibration target — by correlating the motion observed in point cloud registration with the motion measured by the IMU.

The algorithm works while the vehicle is moving: it accumulates LiDAR frames and IMU measurements, solves for the 6-DOF extrinsic transform that best explains both motion sources simultaneously.

## Output

The calibrated transform can be written to the URDF in `eve_description` to make it available over TF for all other nodes.
