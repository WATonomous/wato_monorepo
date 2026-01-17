# carla_perception

Sensor data publishers for CARLA simulation.

## Overview

Spawns virtual sensors in CARLA and publishes their data as standard ROS 2 messages.

## Nodes

### camera_publisher

Spawns cameras attached to the ego vehicle and publishes images with camera info.

**Publications:**
- `~/<camera_name>/image_raw` (sensor_msgs/Image)
- `~/<camera_name>/camera_info` (sensor_msgs/CameraInfo)

### lidar_publisher

Spawns LiDAR sensors and publishes point clouds.

**Publications:**
- `~/<lidar_name>/points` (sensor_msgs/PointCloud2)

### bbox_publisher

Publishes 3D bounding boxes for vehicles and pedestrians in the world.

**Publications:**
- `~/bounding_boxes` (visualization_msgs/MarkerArray)

**Parameters:** See `carla_bringup/config/carla_bridge.yaml`

## Usage

Launched automatically via:
```bash
ros2 launch carla_bringup carla_bridge.launch.yaml
```
