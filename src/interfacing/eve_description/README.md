# Eve Description

This package publishes the Eve vehicle TF tree from a URDF generated from the Xacro files in `urdf/`.

## TF publishing

To start publishing transforms (via `robot_state_publisher`), run:

```sh
ros2 launch eve_description tf_publish.yaml
```

## Nominal pano camera extrinsics (TF)

The 8 roof panorama cameras are defined in the URDF and published as TF frames (all lowercase):

- `camera_pano_nn`
- `camera_pano_ne`
- `camera_pano_ee`
- `camera_pano_se`
- `camera_pano_ss`
- `camera_pano_sw`
- `camera_pano_ww`
- `camera_pano_nw`

For sensor integration, treat `base_link` as the reference frame and query TF at runtime:

- Target frame: `base_link`
- Source frame: `camera_pano_*`

The camera frames are positioned on a circle around `roof_mount` and oriented to face outward (see `urdf/eve_roof_mount.xacro`).

## Nominal lower camera extrinsics (TF)

The 4 lower cameras are also defined in the URDF and published as TF frames (all lowercase):

- `camera_lower_ne`
- `camera_lower_se`
- `camera_lower_sw`
- `camera_lower_nw`

These transforms are **nominal placeholders** intended for early sensor integration. Update the `<origin xyz=... rpy=...>` blocks in `urdf/eve_roof_mount.xacro` (macro `lower_cameras`) once you have measured/calibrated values.
