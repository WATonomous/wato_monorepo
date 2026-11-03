# BEVFusion Crash Diagnosis and Bring-up Notes — 2026-11-03

Setup: WATonomous `perception_bringup_dev` container on `trpro-slurm1` (SLURM node), RTX 3090,
ROS 2 Jazzy, `rmw_zenoh_cpp`, TensorRT 11.4. Bag: `sensors_2026_07_07-22_57_05`.

Problems fixed, in order:

1. [BEVFusion crashes on configure](#1-bevfusion-crashes-on-configure-cuda-arch-mismatch) (CUDA arch mismatch)
2. ["Build errors" after the rebuild](#2-build-errors-after-the-rebuild-only-warnings) (only warnings)
3. [Still looking for the old model folder](#3-node-still-looks-for-the-old-model-folder) (stale installed config)
4. [Rebuilding deleted `.plan` files](#4-rebuilding-the-tensorrt-plan-files)
5. [Bag data never reaches the perception nodes](#5-bag-data-never-reaches-the-perception-nodes-zenoh) (Zenoh)
6. [Foxglove can't connect](#6-foxglove-cant-connect)
7. [Files written in the container aren't visible on the host](#7-files-in-the-container-arent-visible-on-the-host)

---

## 1. BEVFusion crashes on configure (CUDA arch mismatch)

### Symptom

During `perception_bringup` launch, the `perception_container` (`component_container_mt`) dies with
SIGABRT (`exit code -6`) about 2 seconds after `detection_lifecycle_manager` starts configuring
`bevfusion_node`:

```
[ERROR] [component_container_mt-1]: process has died [pid 7909, exit code -6, cmd '/opt/ros/jazzy/lib/rclcpp_components/component_container_mt --ros-args -r __node:=perception_container -r __ns:=/'].
```

The colcon build itself is fine: all 25 packages finished with `rc=0`. The abort message isn't in
`~/.ros/log`, because it goes to the terminal's stderr.

### Root cause

Running `bevfusion_node` standalone and triggering `configure` shows the real error:

```
[BEVFusionCore] All required model files exist. Creating core...
CUDA Runtime error create_frustum_kernel # no kernel image is available for execution on the device,
code = cudaErrorNoKernelImageForDevice [209] in file .../CUDA-BEVFusion/src/bevfusion/camera-geometry.cu:211
```

**The CUDA-BEVFusion library was compiled for the wrong GPU architecture.**

- The GPU is an **RTX 3090, which is compute capability 8.6 (`sm_86`)**.
- `src/perception/bevfusion/cuda_bevfusion_vendor/CMakeLists.txt:72-73` falls back to `CUDASM=89`
  (RTX 40-series / Ada) when `CUDASM` isn't set, and it wasn't set.
- `src/wato-cuda-bevfusion/CUDA-BEVFusion/CMakeLists.txt:48` compiles with
  `-gencode arch=compute_89,code=compute_89`, which produces only `compute_89` PTX and no SASS for any
  other GPU. PTX for 8.9 can't run on an 8.6 device, so the first kernel launch fails.

### Fix (verified)

Clear the vendor build, because `CUDASM=89` is saved in the generated ExternalProject build files:

```bash
cd /ws
rm -rf build/cuda_bevfusion_vendor install/cuda_bevfusion_vendor
export CUDASM=86
colcon build --packages-up-to bevfusion
```

To verify it worked:

```bash
cuobjdump --list-ptx --list-elf install/cuda_bevfusion_vendor/opt/cuda_bevfusion_vendor/lib/libbevfusion_core.so | grep -o 'sm_[0-9]*' | sort -u
# should print sm_86
```

After the rebuild, standalone configure printed `Node configured successfully` and `Node activated successfully!`.

To stop this happening again:

- Put `export CUDASM=86` in your shell profile or dev-container env. On SLURM, also pin `GPU_TYPE` in
  `session_config.sh` so you always get the same GPU type.
- Or detect the arch in `cuda_bevfusion_vendor/CMakeLists.txt` from
  `nvidia-smi --query-gpu=compute_cap`, or emit SASS for several archs, e.g.
  `-gencode arch=compute_86,code=sm_86 -gencode arch=compute_89,code=sm_89`.

### Related: why Ctrl-C hung

`detection_lifecycle_manager` was still waiting on the configure call to the dead node
(`transition_timeout_s: 130.0`), so it ignored SIGINT and SIGTERM and had to be SIGKILLed. This is a
side effect of the crash, not a separate bug.

---

## 2. "Build errors" after the rebuild (only warnings)

colcon prints a yellow `Finished <<< cuda_bevfusion_vendor ... -- stderr output` summary whenever a
package writes anything to stderr. That looks like a failure but isn't. Here it was about 60 lines
of warnings:

- TensorRT `warning #997-D` about deprecated `IPluginV2` methods in `custom_layernorm.cu`
- signed/unsigned comparison warnings in `lidar-scn-onnx-parser.cpp`
- `jobserver unavailable` and CMake dev warnings

To check whether a build actually failed, look at the return codes rather than the colors:

```bash
grep JobEnded log/latest_build/events.log | grep -v "'rc': 0"   # no output = nothing failed
```

---

## 3. Node still looks for the old model folder

The model folder was renamed from `resnet50int8_trt11` to `resnet50_trt11_int8`, which matches the
defaults in `bevfusion/config/params.yaml` and `bevfusion_node.cpp:80`. `model_dir` was updated in
`src/perception/perception_bringup/config/perception_bringup.yaml`, but the node still used the old path.

**Cause:** launch reads the *installed* copy,
`install/perception_bringup/share/perception_bringup/config/perception_bringup.yaml`. The workspace
isn't built with `--symlink-install`, so edits in `src/` only reach `install/` when you rebuild that package.

**Fix:**

```bash
colcon build --packages-select perception_bringup
```

If you build with `colcon build --symlink-install`, config and launch edits in `src/` take effect
without a rebuild.

---

## 4. Rebuilding the TensorRT plan files

There's no separate build tool. When `bevfusion_node` configures, it checks `<model_dir>/build/` for
the four plans (`camera.backbone`, `camera.vtransform`, `fuser`, `head.bbox`). If any are missing,
it builds them from the `.onnx` files in `model_dir`.

Plans are tied to the GPU and TensorRT version they were built on. If you change GPU type or TensorRT
version, or see deserialize errors, delete `build/` and let the node rebuild them.

**Gotcha: permissions.** `resnet50_trt11_int8/` is owned by root on the shared `wato-home` storage.
If you delete `build/`, your user can't recreate it, and configure fails with:

```
[BEVFusionCore] ERROR: Failed to create build directory: Permission denied
```

**Steps:**

```bash
# 1. recreate a writable build folder
sudo mkdir -p /opt/watonomous/models/bevfusion/resnet50_trt11_int8/build
sudo chmod 777 /opt/watonomous/models/bevfusion/resnet50_trt11_int8/build

# 2. build the engines once standalone. This takes several minutes (INT8), which may exceed the
#    lifecycle manager's 130 s timeout if you let bringup do it.
ros2 run bevfusion bevfusion_node --ros-args \
  --params-file /ws/install/perception_bringup/share/perception_bringup/config/perception_bringup.yaml
# in a second terminal:
ros2 lifecycle set /bevfusion_node configure
# wait for "Node configured successfully", then Ctrl-C

# 3. launch bringup as usual
ros2 launch perception_bringup perception.launch.yaml live_cameras:=false
```

---

## 5. Bag data never reaches the perception nodes (Zenoh)

### Symptom

`./watod bag play ... --loop` runs and prints:

```
[WARN] [rmw_zenoh_cpp]: Unable to connect to a Zenoh router. Have you started a router with `ros2 run rmw_zenoh_cpp rmw_zenohd`?
... Proceeding with initialization but other peers will not discover or receive data from peers in this session until a router is started.
```

The topics show up in `ros2 topic list`, but only because the perception nodes subscribe to them.
`ros2 topic info /lidar_cc/velodyne_points` shows **`Publisher count: 0`**, and `ros2 topic echo --once`
receives nothing.

### Cause

With `rmw_zenoh`, nodes only find each other through a Zenoh router. The session config
(`ZENOH_SESSION_CONFIG_URI=/opt/watonomous/rmw_zenoh_session_config.json5`) has multicast scouting
disabled and connects only to `tcp/localhost:7447`. `watod bag play` runs the player in its own
container, and that container didn't reach the router used by the perception stack. The router
comes with `infrastructure_bringup` from `./watod up`.

### Fix

- Run `./watod up` first, so `infrastructure_bringup` (router and Foxglove bridge) is running, *then* start the bag.
- Or play the bag from inside the dev container, which can reach the router:

  ```bash
  ros2 bag play /ws/bags/sensors_2026_07_07-22_57_05/sensors_2026_07_07-22_57_05_0.mcap --loop
  ```

Check that the data is flowing: `ros2 topic info /lidar_cc/velodyne_points` should show `Publisher count: 1`.

### Harmless warnings in the same output

- `error setting scheduling priority ... can be safely ignored`: the container can't raise thread priority.
- `stdin is not a terminal device. Keyboard handling disabled`: you just can't pause or seek with the keyboard.

---

## 6. Foxglove can't connect

`./watod up` prints the ports:

```
ssh -L 41422:localhost:41422 -L 41426:localhost:41426 -L 41427:localhost:41427 <host>
Foxglove:    ws://10.0.50.182:41422
Log Viewer:  http://10.0.50.182:41426
```

### What went wrong

- Foxglove runs on the laptop, so `127.0.0.1` there is the laptop, not the node.
- `ssh ... trpro-slurm1` failed with `Could not resolve hostname`. That name only resolves inside the
  cluster; use the SSH alias or jump host you normally connect with.
- VS Code Remote-SSH had **already auto-forwarded port 41422** (`lsof -nP -iTCP:41422 -sTCP:LISTEN`
  showed `Code Helper`). Because of that, a manual `ssh -L 41422:...` can't bind the port. The VS Code
  forward was also **stale**: it accepted connections but passed 0 bytes and timed out.

### Test the bridge with curl

Run this on the node first, then on the laptop:

```bash
curl -i -m 5 --output - -H "Connection: Upgrade" -H "Upgrade: websocket" \
  -H "Sec-WebSocket-Version: 13" -H "Sec-WebSocket-Key: dGhlIHNhbXBsZSBub25jZQ==" \
  -H "Sec-WebSocket-Protocol: foxglove.sdk.v1" http://127.0.0.1:41422/
```

A healthy reply is `HTTP/1.1 101 Switching Protocols` followed by a `{"op":"serverInfo",...}` JSON
message (foxglove-bridge 3.6.0, ROS Jazzy).

- Works on the node but times out on the laptop: the port forward is the problem.
- Fails on the node too: the bridge or `infrastructure_bringup` isn't running.

### Fix (what worked)

1. In VS Code, open the **Ports** tab, right-click `41422` → **Stop Forwarding**, then **Forward a Port** → `41422`.
2. Rerun the curl on the laptop and check for `101` and `serverInfo`.
3. In Foxglove: Open connection → Foxglove WebSocket → `ws://localhost:41422`. Use `ws://`, not `wss://`.
   If the browser version refuses, use the desktop app.

If the forward drops again, for example after VS Code reconnects, repeat step 1.

Alternative that doesn't depend on VS Code:

```bash
ssh -N -L 18765:localhost:41422 <your-ssh-alias>   # then connect Foxglove to ws://localhost:18765
```

### Viewing detections

Add a **3D** panel, set the display frame to `base_link`, and turn on:

- `/lidar_cc/velodyne_points`: the point cloud
- `/perception/bev_detection_markers`: detection boxes
- `/perception/detections_3d_bev`: raw `Detection3DArray`

---

## 7. Files in the container aren't visible on the host

Only these folders are bind-mounted from the host. Everything else in the container, including
`/ws/build`, `/ws/install` and `/ws/log`, exists only inside it.

| Container | Host |
|---|---|
| `/ws/src/perception` | `~/src/wato_monorepo/src/perception` |
| `/ws/bags` | `~/src/wato_monorepo/bags` |
| `/opt/watonomous/models` | `~/src/wato_monorepo/models` |

The mounted folders are owned by root, so writing into them from the container needs `sudo`. To
copy a file out of a folder that isn't mounted, run this on the host:
`docker cp <container>:/ws/<path> .`

If `/opt/watonomous/models` looks empty in the container after the host's `models/` was deleted and
recreated, the bind mount is stale. Run `./watod down` and then `./watod up`.

---

## Diagnostic commands

| What | Command |
|---|---|
| Did any package fail to build? | `grep JobEnded log/latest_build/events.log \| grep -v "'rc': 0"` |
| Runtime logs | `~/.ros/log/latest/launch.log`, `~/.ros/log/*.log` |
| See errors that don't reach the logs | run the node standalone: `ros2 run bevfusion bevfusion_node --ros-args --params-file <yaml>`, then `ros2 lifecycle set /bevfusion_node configure` |
| GPU and compute capability | `nvidia-smi --query-gpu=name,compute_cap --format=csv` |
| Which archs a `.so` was built for | `cuobjdump --list-ptx --list-elf <lib.so>` |
| Is a topic actually published? | `ros2 topic info <topic>` (check `Publisher count`) |
| Zenoh settings | `env \| grep -i zenoh` |
| Who holds a port (laptop) | `lsof -nP -iTCP:<port> -sTCP:LISTEN` |
