# Lanelets and SemVecNet

## Build

Install `CUDA-SemVecNet` from the `wato-SemVecNet-cpp` checkout that exposes
`semvecnet::Polylines` and accepts `lidar_to_map`, `ego_to_map`, and the optional
`semantic_rgb` output in `SemVecNet::forward`.
`lanelets/CMakeLists.txt` already uses `find_package(semvecnet REQUIRED CONFIG)`
and links `semvecnet::semvecnet_core`. Install the external project in the
perception environment; there's no need to copy it into the ROS package.
The node uses `lanelet2_core` (`ros-jazzy-lanelet2-core` on Jazzy) and the
monorepo's `lanelet_msgs`. Use the same CUDA, TensorRT, and GPU ONNX Runtime SDK
that the perception node will run with. For example, in the perception dev container
after `onnxruntime_gpu_vendor` and the other ROS dependencies have been built:

```sh
source /ws/install/local_setup.bash
cmake -S /path/to/wato-SemVecNet-cpp/CUDA-SemVecNet -B /tmp/semvecnet-build \
  -DCMAKE_INSTALL_PREFIX=/ws/semvecnet-install \
  -DCMAKE_CUDA_ARCHITECTURES=86 \
  -DONNXRUNTIME_ROOT=/ws/install/onnxruntime_gpu_vendor \
  -DCMAKE_INSTALL_RPATH=/ws/install/onnxruntime_gpu_vendor/lib \
  -DBUILD_TESTING=OFF
cmake --build /tmp/semvecnet-build
cmake --install /tmp/semvecnet-build
colcon build --packages-select lanelets --cmake-args \
  -Dsemvecnet_DIR=/ws/semvecnet-install/lib/cmake/semvecnet \
  -DONNXRUNTIME_ROOT=/ws/install/onnxruntime_gpu_vendor
colcon build --packages-select perception_bringup
```

Set `CMAKE_CUDA_ARCHITECTURES` for your GPU (`86` for Ampere, `89` for Ada).
The current stock perception image does not install this external library.

## Replay a bag with Foxglove

Put `hrnet_ocrnet_mapillary.plan` and `semvecnet.onnx` in `models/semvecnet/`
on the host. Both watod perception modes mount `models/` at
`/opt/watonomous/models`. Use a TensorRT plan compatible with the container's
TensorRT version and GPU.

**Development:** with `ACTIVE_MODULES="perception:dev"` in `watod-config.local.sh`,
run `./watod up` and open a shell with `./watod -t perception_bringup_dev`.
The dev container doesn't autostart ROS nodes. In it, run:

```sh
source /ws/install/local_setup.bash
export LD_LIBRARY_PATH="/ws/install/onnxruntime_gpu_vendor/lib:${LD_LIBRARY_PATH:-}"
ros2 launch perception_bringup perception.launch.yaml \
  live_cameras:=false enable_lanelets:=true use_sim_time:=true
```

If the models are elsewhere, add `params_file:=/path/to/lanelets.yaml`
with `hrnet_plan` and `semvecnet_onnx` pointing to the files in the container.

**Deployment (no container shell):** add the following to `watod-config.local.sh`:

```sh
export ACTIVE_MODULES="perception"
export PERCEPTION_BUILD_TARGET=deploy_semvecnet
export SEMVECNET_SOURCE_DIR="$HOME/Desktop/code/wato-SemVecNet-cpp/CUDA-SemVecNet"
export PERCEPTION_ENABLE_LANELETS=true
export PERCEPTION_LIVE_CAMERAS=false
export PERCEPTION_USE_SIM_TIME=true
```

Run `./watod -m perception build` once, and then `./watod up -d`. The optional
deploy target builds the external SDK from `SEMVECNET_SOURCE_DIR` and packages
the lanelets node into the image. The stock `deploy` target does not require
the external checkout or include the lanelets node.

After the node logs `Lanelets preprocessing ready`, run
`./watod bag play <bag> --clock` from the host. In Foxglove, connect to
the WebSocket URL printed by `watod up`, open a **3D** panel, set its fixed
frame to `map`, and enable `/perception/semvecnet_polyline_markers`. The
repo's `config/foxglove_config/track_testing.json` layout already enables it.
Import `config/foxglove_config/Panorama_SemVecNet.json` to keep Panorama's
camera arrangement and get a separate top-down polyline panel under the
LiDAR/BEVFusion view. That panel is a Foxglove **Image** panel on
`/perception/semvecnet_preview/compressed`, not a 3D marker view. It
needs no camera calibration or display-frame transform. The camera panels use
the vehicle bag's compressed topics. Red is boundary, green is centerline,
gold is divider, and blue is pedestrian crossing.

If the ROS nodes log Fast CDR deserialization errors during playback, disable
Zenoh shared-memory transport for the bag player and try a slower rate:

```sh
ZENOH_CONFIG_OVERRIDE=transport/shared_memory/enabled=false \
  ./watod bag play <bag> --clock --rate 0.5
```

For a standalone launch (when `multi_camera_sync` is already running or its
outputs are in the bag), run `ros2 launch lanelets lanelets.launch.yaml use_sim_time:=true`.
The launch defaults use the same six
rectified camera streams and `/lidar_cc/velodyne_points` as BEVFusion.
To use the merged cloud, pass `cloud_topic:=/lidar/all/points_merged`.
For raw batched images, use a params file with `use_compressed_images: false`
and run `multi_camera_sync` in raw mode.

TF must provide transforms from the cloud's `header.frame_id` to each camera's
optical frame and to `map_frame` at the cloud timestamp, plus `ego_frame` to
`map_frame` at that same time. `ego_frame` defaults to `rear_axle`, matching
the offline preview. Frames without calibration,
TF, or valid LiDAR points are skipped. If deploying with bag playback, play the
transforms alongside the sensors and use simulation time when appropriate.

`LaneletsNode::processFrame` calls `SemVecNet::forward` with packed RGB8 images in
`camera_names_6` / `camera_names_8` order (selected by `num_cameras`), FP16 `[x, y, z, intensity, time_lag]` LiDAR records (zero lag
for the current sweep), per-camera calibration relative to the input image
size (the SDK handles the HRNet mask scaling), and the LiDAR and ego poses in
the map frame.

## Output

Each successful inference publishes a PNG on
`/perception/semvecnet_preview/compressed` (`sensor_msgs/msg/CompressedImage`).
It follows the offline C++ demo's `combined/` output: the SDK's semantic input
map on the left and the class-colored rear-axle vectors, grid, and vehicle
outline on the right. The preview uses the same coordinate conversion and
forward offset as the demo. Live sensor selection and synchronization differ
from the offline eight-camera/merged-cloud runner, so individual predictions
are not expected to match frame for frame.

Every successful inference publishes `/perception/semvecnet_polyline_markers`
(`visualization_msgs/msg/MarkerArray`), independently of lanelet generation.
Each predicted polyline becomes a color-coded `LINE_STRIP` in `map_frame`
at the cloud timestamp. The marker namespaces identify dividers, pedestrian
crossings, boundaries, and centerlines. Each array clears the previous snapshot,
including when the model returns no polylines.

When `generateLanelets` returns geometry, the node serializes it to
`lanelet_msgs/msg/MapVisualization` (a stamped array of `Lanelet` messages)
on `/perception/detected_lanelets`. The header uses the cloud timestamp and
`map_frame`. This is perception geometry, not a routable map: no topology,
traffic rules, speed limits, or lane-change permissions are inferred. The
publisher is registered, but the empty generator stub sends no messages.
