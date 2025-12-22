## Intrinsics Calibration

For a simple monocular calibration, run the cameracalibrator node.

```sh
CAMERA=camera_pano_nn
ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.08686 --no-service-check --ros-args --remap image:=/$CAMERA/image_raw -p camera:=/$CAMERA
```

Later releases will use a tool to calibrate multiple sensors at once.
