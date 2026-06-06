# Developing eve_description

## Build & Launch

```bash
colcon build --packages-select eve_description
ros2 launch eve_description tf_publish.yaml
```

## Updating Camera Extrinsics

Lower camera transforms are placeholders. To update them once calibrated:

1. Open `urdf/eve_roof_mount.xacro`.
2. Find the `lower_cameras` macro block.
3. Update the `<origin xyz="..." rpy="..."/>` values for each camera.
4. Rebuild and relaunch — TF updates immediately.

Panorama camera extrinsics are in the same file under the `pano_cameras` macro. These come from the camera calibration pipeline (`calibration/camera_calib`).

## Adding a New Sensor Frame

1. Add a `<link>` and `<joint>` block in the appropriate Xacro file under `urdf/`.
2. Set `<joint type="fixed">` with a measured or calibrated `<origin>`.
3. Set the parent to the nearest physically meaningful frame (`base_link`, `roof_mount`, etc.).
4. Rebuild — `robot_state_publisher` will broadcast the new frame automatically.

## Internal Architecture

`eve_description` is a data-only package — there is no executable node. Xacro files are expanded by `robot_state_publisher` into a URDF string, which is then broadcast as `/tf_static` transforms. All joints are `fixed`, so transforms are published once on startup and never change during runtime.

If the URDF has a syntax error, `robot_state_publisher` will fail to start and no TF frames will be published — check the node log for Xacro parse errors.

## After Launching

Verify the full TF tree is being published:

```bash
ros2 run tf2_tools view_frames
# Opens frames.pdf showing the complete tree
```

Or check individual frames:

```bash
ros2 topic echo /tf_static --once
```

## Definition of Good Result

All 14 sensor frames must be present in the TF tree:

```
base_link
├── rear_axle
├── front_axle
└── roof_mount
    ├── camera_pano_nn / ne / ee / se / ss / sw / ww / nw  (8 frames)
    └── camera_lower_ne / se / sw / nw                      (4 frames)
```

Check any frame is reachable from `base_link`:

```bash
ros2 run tf2_ros tf2_echo base_link camera_pano_nn
# Should print a valid transform, not an error
```

If frames are missing, the Xacro file has a syntax error or `robot_state_publisher` did not receive the robot description parameter.

## Frame Naming Convention

Compass directions use two-letter codes: `nn` (north), `ne` (northeast), `ee` (east), `se` (southeast), `ss` (south), `sw` (southwest), `ww` (west), `nw` (northwest). All frame names are lowercase.
