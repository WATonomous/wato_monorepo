# lidar_lidar_calib

LiDAR to LiDAR calibration package. Takes a target cloud and source cloud, and computes a transform with GICP.

> Note: this method requires a pretty good initial guess. This can be obtained using physical measurements. You know that you've made a good initial guess when this method starts to converge on a solution consistently. Further work can be made on automating finding a good initial guess with something like a Particle Filter.

## Usage
This method takes in two pointcloud topics, logs and publishes a transformation between the two pointclouds. For setting configurations like initial guess and max corrospondence distance, refer to the sample [config file](./config/config.yaml).

```bash
colcon build --packages-select lidar_lidar_calib
source install/setup.bash

ros2 launch lidar_lidar_calib lidar_lidar_calib.yaml source_topic:="/source_pointcloud_topic" target_topic:="/target_pointcloud_topic"
```
