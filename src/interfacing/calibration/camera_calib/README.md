# camera_calib

Camera intrinsic and extrinsic calibration data for all cameras on Eve, and the tools to generate it.

## Overview

Before cameras can be used for perception, two calibration steps are needed:

1. **Intrinsics** — lens distortion coefficients and focal length for each individual camera, computed from a checkerboard target.
2. **Extrinsics** — the transform from each camera frame to the vehicle (`base_link`), stored in `eve_description` as URDF/Xacro and broadcast over TF.

This package stores the calibration YAML files output by the intrinsics calibration step, and provides instructions for running calibration.

## Cameras

| Group | Count | Frame names |
|-------|-------|-------------|
| Roof panorama | 8 | `camera_pano_nn/ne/ee/se/ss/sw/ww/nw` |
| Lower side | 4 | `camera_lower_ne/se/sw/nw` |
