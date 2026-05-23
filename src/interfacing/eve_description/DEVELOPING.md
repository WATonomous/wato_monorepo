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

## Frame Naming Convention

Compass directions use two-letter codes: `nn` (north), `ne` (northeast), `ee` (east), `se` (southeast), `ss` (south), `sw` (southwest), `ww` (west), `nw` (northwest). All frame names are lowercase.
