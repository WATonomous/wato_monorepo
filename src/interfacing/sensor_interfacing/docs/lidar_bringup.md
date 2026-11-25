# LiDAR Bringup

Eve has three Velodyne lidars: a VLS-128 on the center roof mount (`lidar_cc`) and two VLP-16s
(`lidar_ne`, `lidar_nw`). To publish raw frames from all three, run:

```sh
ros2 launch sensor_interfacing all_lidars_composed.yaml
```

## Visualization

To visualize in Foxglove without a URDF, we need a reference frame. This will be added in a future PR that contains publishers for sensor extrinsics.

```sh
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 lidar_cc base_link
```
