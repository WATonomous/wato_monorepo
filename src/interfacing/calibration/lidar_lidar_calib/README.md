# lidar_lidar_calib

LiDAR to LiDAR extrinsic calibration package. Takes a target cloud and source cloud, and computes a transform with GICP.

> Note: this method requires a pretty good initial guess. This can be obtained using physical measurements. You know that you've made a good initial guess when this method starts to converge on a solution consistently. Further work can be made on automating finding a good initial guess with something like a Particle Filter.

## Usage
This method takes in two pointcloud topics, logs and publishes a transformation between the two pointclouds. For setting configurations like initial guess and max corrospondence distance, refer to the sample [config files](./config).

### Prerequisites
- Make sure robot is stationary
- Make sure robot is in a room that is feature-rich (that is, the room is not empty and a perfect square)

```bash
colcon build --packages-select lidar_lidar_calib
source install/setup.bash

ros2 launch lidar_lidar_calib lidar_lidar_calib.yaml
```

This launches two nodes, each calibrating between the center lidar of the car and one of the mirror mounted lidars.
