# Developing camera_calib

## Intrinsic Calibration

Uses the ROS `camera_calibration` package with a printed checkerboard target (7×10 inner corners, 86.86 mm square size).

```bash
CAMERA=<camera_frame_name>
ros2 run camera_calibration cameracalibrator \
  --size 7x10 \
  --square 0.08686 \
  --no-service-check \
  --ros-args \
  --remap image:=/$CAMERA/image_raw \
  -p camera:=/$CAMERA
```

Move the checkerboard through the camera's full field of view (tilted, rotated, at different distances) until the calibration tool indicates sufficient coverage, then click **Calibrate** and **Save**. Copy the resulting YAML into this package.

## Calibration File Format

Each YAML file follows the ROS `camera_info` format:

```yaml
image_width: 1920
image_height: 1200
camera_name: camera_pano_nn
camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx, 0, fy, cy, 0, 0, 1]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]
rectification_matrix: ...
projection_matrix: ...
```

## Extrinsic Calibration

Camera-to-vehicle extrinsics (position and orientation relative to `base_link`) are stored in `eve_description/urdf/eve_roof_mount.xacro`. After measuring or calibrating camera positions, update the `<origin xyz=... rpy=.../>` blocks there. See [eve_description/DEVELOPING.md](../../eve_description/DEVELOPING.md) for details.
