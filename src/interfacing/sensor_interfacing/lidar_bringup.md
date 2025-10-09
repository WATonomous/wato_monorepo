# LiDAR Bringup

To publish raw frames from the roof-mounted VLP-32C, run:

```sh
ros2 launch interfacing_bringup velodyne_VLP32C.launch.py
```
## Visualization

To visualize in Foxglove without a URDF, we need a reference frame. This will be added in a future PR that contains publishers for sensor extrinsics.

```sh
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 velodyne base_link
```

## Additional Links

https://icave2.cse.buffalo.edu/resources/sensor-modeling/VLP32CManual.pdf