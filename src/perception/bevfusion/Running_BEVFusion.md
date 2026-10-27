# Running BEVFusion

How to run BEVFusion on a rosbag and view its detections in Foxglove. For how the node works internally, see [DEVELOPING.md](./DEVELOPING.md).

BEVFusion needs an NVIDIA GPU (TensorRT + the `nvidia` Docker runtime), so it won't run on a Mac. Use a WATcloud SLURM dev session (see [wato_asd_tooling](https://github.com/WATonomous/wato_asd_tooling)) or another GPU machine.

## 1. Get the model weights

Model weights are not in this repo. They are in the [Weights Google Drive folder](https://drive.google.com/drive/folders/10EXFocqX40L4JLPhppSyHl9GuhrG9wPj) (WATonomous members only).

Docker-compose mounts `wato_monorepo/models` → `/opt/watonomous/models`, and the node reads `model_dir` from [`perception_bringup.yaml`](../perception_bringup/config/perception_bringup.yaml):

```
/opt/watonomous/models/bevfusion/resnet50_trt11_int8
```

So the files have to end up at:

```
wato_monorepo/models/bevfusion/resnet50_trt11_int8/
    ├── *.onnx                      (camera.backbone, camera.vtransform, fuser, head.bbox)
    ├── lidar.backbone.xyz.onnx
    └── build/                      (.plan TensorRT engines, built automatically on first run)
```

> [!NOTE]
> The Drive folder may be called `resnet50int8_trt11`. Either rename it to `resnet50_trt11_int8` when you copy it (as shown below), or change `model_dir` in `perception_bringup.yaml` to match.

### Copying the weights to WATcloud / a SLURM node

Your WATcloud home directory is shared between the login nodes and the SLURM nodes. Download the folder from Drive on your laptop, unzip it, and push it through a login node:

```bash
ssh -i <your_watcloud_key> <watcloud_user>@wato-login2 "mkdir -p <path_to_wato_monorepo>/models/bevfusion"

rsync -avP -e "ssh -i <your_watcloud_key>" \
  ./resnet50int8_trt11/ \
  <watcloud_user>@wato-login2:<path_to_wato_monorepo>/models/bevfusion/resnet50_trt11_int8/
```

- Leave a space between the source and the destination. Without it, rsync reads both as one local path and fails with `(l)stat: No such file or directory`.
- The trailing `/` on the source copies the folder's *contents*, so the destination name decides the final folder name.
- Replace `<path_to_wato_monorepo>` with where your clone actually is (run `pwd` in it on WATcloud, e.g. `~/src/wato_monorepo`). If the path is wrong, the containers won't see the model.

You don't need to restart the SLURM session: the files are visible on the node right away. Check with:

```bash
ls <path_to_wato_monorepo>/models/bevfusion/resnet50_trt11_int8
```

## 2. Start perception

In `watod-config.local.sh`:

```bash
export ACTIVE_MODULES="perception:dev"
```

Then:

```bash
./watod up   # also starts infrastructure (Foxglove bridge); prints the Foxglove URL and an ssh -L command
```

Open a shell in the `perception_bringup_dev` container and run:

```bash
colcon build --packages-up-to perception_bringup bevfusion
source install/setup.bash
ros2 launch perception_bringup perception.launch.yaml live_cameras:=false
```

- Set `live_cameras:=false` when playing a bag. Otherwise the launch tries to start the physical Hikrobot camera drivers.
- **The first launch takes several minutes** while TensorRT builds the `.plan` engines into `models/bevfusion/resnet50_trt11_int8/build/`. Later launches reuse them.

### If the model was copied while containers were already running

The `models/` bind mount is live, so new files should show up inside the container. Check:

```bash
ls /opt/watonomous/models/bevfusion/resnet50_trt11_int8   # inside the container
```

- **Files are there:** restart the `ros2 launch` (Ctrl+C, then run it again). The node only loads the model when it starts.
- **Directory is empty:** the bind mount is stale, usually because `models/` was deleted and recreated. Run `watod down` then `watod up`.

## 3. Play a bag

Bags are in the [Bag Recordings Google Drive folder](https://drive.google.com/drive/folders/127Kw3o7Org474rkK1wwMDRZKiYHaR0Db). Put them in `wato_monorepo/bags/`. On WATcloud, set `BAG_DIRECTORY` to a folder in `/mnt/wato-drive2/` to play bags without copying them.

```bash
./watod bag ls
./watod bag play <bag_dir>/<bag>_0.mcap --loop
```

The bag must contain:

- the 6 `camera_pano_*` camera feeds (they go through `multi_camera_sync`)
- `/lidar_cc/velodyne_points` (or change `input_lidar_topic` in `perception_bringup.yaml`; a launch-time remap won't work)
- `/tf_static` with the camera → `lidar_cc` transforms. Without these, calibration never initializes and you get no detections.

## 4. View in Foxglove

1. If you're running on a remote machine or SLURM node, run the `ssh -L ...` command printed by `watod up` on your laptop first.
2. In Foxglove: **Open connection → Foxglove WebSocket →** `ws://localhost:<FOXGLOVE_BRIDGE_PORT>`.
3. Add a **3D** panel and set the display frame to `base_link`.
4. Turn on these topics:

| Topic | Type | What it shows |
|---|---|---|
| `/perception/bev_detection_markers` | `visualization_msgs/MarkerArray` | Detection boxes, colored by class |
| `/perception/detections_3d_bev` | `vision_msgs/Detection3DArray` | The raw detections |
| `/lidar_cc/velodyne_points` | `sensor_msgs/PointCloud2` | The point cloud, for context |

## Troubleshooting

| Symptom | Likely cause |
|---|---|
| Node fails on startup and complains about missing model files | `model_dir` doesn't match the folder name under `models/bevfusion/`. |
| TensorRT engine deserialize errors | The `.plan` engines in `build/` were built on a different GPU or TensorRT version. Delete `build/` and let them rebuild. On SLURM, pin `GPU_TYPE` in `session_config.sh` so you get the same GPU type each session. |
| CUDA out of memory | Not enough GPU memory. Raise `VRAM` in `session_config.sh`. |
| No boxes, no `[PROFILER]` lines in the logs | Inputs aren't syncing or calibration isn't set up. Check that the camera, LiDAR, camera_info and `/tf_static` topics are all in the bag. |
| `[PROFILER]` lines appear but no boxes in Foxglove | Check the 3D panel's display frame (`base_link`) and that the markers topic is turned on. |
