# Lanelets and SemVecNet

## Build

Install `CUDA-SemVecNet` from the `wato-SemVecNet-cpp` checkout that exposes
`semvecnet::Polylines` and accepts both `lidar_to_map` and `ego_to_map`.
The node uses `lanelet2_core` (`ros-jazzy-lanelet2-core` on Jazzy) and the
monorepo's `lanelet_msgs`. Use the same CUDA, TensorRT, and GPU ONNX Runtime SDK
that the perception node will run with. For example, in the perception dev container:

```sh
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
```

Set `CMAKE_CUDA_ARCHITECTURES` for your GPU (`86` for Ampere, `89` for Ada).
The current stock perception image does not install this external library.

## Run

Set `hrnet_plan` and `semvecnet_onnx` in the launch `params_file` to the
installed HRNet TensorRT plan and SemVecNet ONNX model. The defaults are under
`/opt/watonomous/models/semvecnet`.

Run `ros2 launch lanelets lanelets.launch.yaml` with the perception workspace sourced.
If both CPU and GPU ONNX Runtime are installed, put the GPU build first before
launching: `export LD_LIBRARY_PATH=/ws/install/onnxruntime_gpu_vendor/lib:$LD_LIBRARY_PATH`.
Like BEVFusion, it takes batched rectified images and camera info from `multi_camera_sync`
and approximate-time synchronizes images with LiDAR. The launch defaults use the same
six cameras and `/lidar_cc/velodyne_points` as BEVFusion. To use the merged cloud, pass
`cloud_topic:=/lidar/all/points_merged`. For raw batched images, use a `params_file`
with `use_compressed_images: false` and run `multi_camera_sync` in raw mode.

TF must provide transforms from the cloud's `header.frame_id` to each camera's
optical frame and to `map_frame` at the cloud timestamp, plus `ego_frame` to
`map_frame` at that same time. `ego_frame` defaults to `base_link`. Frames without calibration,
TF, or valid LiDAR points are skipped. If deploying with bag playback, play the
transforms alongside the sensors and use simulation time when appropriate.

`LaneletsNode::processFrame` calls `SemVecNet::forward` with packed RGB8 images in
`camera_names` order, FP16 `[x, y, z, intensity, time_lag]` LiDAR records (zero lag
for the current sweep), per-camera calibration scaled to 960×720, and the
LiDAR and ego poses in the map frame. A successful call passes `semvecnet::Polylines`
to `generateLanelets`, which is still a stub. A polyline has a
class label (divider, pedestrian crossing, boundary, or centerline), a score,
and 20 ordered `(x, y)` points in meters.

## Output

When `generateLanelets` returns geometry, the node serializes it to
`lanelet_msgs/msg/MapVisualization` (a stamped array of `Lanelet` messages)
on `/perception/detected_lanelets`. The header uses the cloud timestamp and
`map_frame`. This is perception geometry, not a routable map: no topology,
traffic rules, speed limits, or lane-change permissions are inferred. The
publisher is registered, but the empty generator stub sends no messages.
