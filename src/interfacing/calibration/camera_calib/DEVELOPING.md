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

## After Calibrating

1. **Move the checkerboard** through the full field of view — tilt it left/right, up/down, bring it close and far. The calibration tool shows four progress bars (X, Y, size, skew). Fill all four bars before calibrating.

2. **Click Calibrate** in the calibration tool GUI. Processing may take 10–30 seconds.

3. **Click Save** to write the YAML to `/tmp/calibrationdata.tar.gz`. Extract and rename the file to match the camera frame name (e.g. `camera_pano_nn.yaml`) and copy it into this package.

4. **Verify the saved file** has all required fields (`camera_matrix`, `distortion_coefficients`, `rectification_matrix`, `projection_matrix`).

## Definition of Good Result

The calibration tool reports a **reprojection error** after clicking Calibrate:

| Reprojection error | Quality |
|--------------------|---------|
| < 0.25 pixels | Excellent |
| 0.25 – 0.5 pixels | Good — suitable for use |
| 0.5 – 1.0 pixels | Acceptable — recalibrate if possible |
| > 1.0 pixels | Poor — discard and recalibrate with more coverage |

If reprojection error is high:
- Ensure the checkerboard is flat (warped boards give poor results)
- Use more images with varied orientations and distances
- Ensure the checkerboard fills at least 30% of the frame in some images

## Extrinsic Calibration

Camera-to-vehicle extrinsics (position and orientation relative to `base_link`) are stored in `eve_description/urdf/eve_roof_mount.xacro`. After measuring or calibrating camera positions, update the `<origin xyz=... rpy=.../>` blocks there. See [eve_description/DEVELOPING.md](../../eve_description/DEVELOPING.md) for details.
