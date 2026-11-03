# BEVFusion Crash Diagnosis — 2026-11-03

## Symptom

During `perception_bringup` launch, the `perception_container` (`component_container_mt`) dies with
SIGABRT (`exit code -6`) about 2 seconds after `detection_lifecycle_manager` starts configuring
`bevfusion_node`:

```
[ERROR] [component_container_mt-1]: process has died [pid 7909, exit code -6, cmd '/opt/ros/jazzy/lib/rclcpp_components/component_container_mt --ros-args -r __node:=perception_container -r __ns:=/'].
```

The colcon build itself is fine: all 25 packages finished with `rc=0` and only harmless warnings.

## Root cause

Running `bevfusion_node` standalone and triggering `configure` shows the error that never made it
into the log files:

```
[BEVFusionCore] All required model files exist. Creating core...
CUDA Runtime error create_frustum_kernel # no kernel image is available for execution on the device,
code = cudaErrorNoKernelImageForDevice [209] in file .../CUDA-BEVFusion/src/bevfusion/camera-geometry.cu:211
```

**The CUDA-BEVFusion library was compiled for the wrong GPU architecture.**

- The machine's GPU is an **NVIDIA GeForce RTX 3090, which is compute capability 8.6 (`sm_86`)**.
- `src/perception/bevfusion/cuda_bevfusion_vendor/CMakeLists.txt:72-73` falls back to
  `CUDASM=89` (RTX 40-series / Ada) when `CUDASM` isn't set in the environment, and it wasn't set.
- `src/wato-cuda-bevfusion/CUDA-BEVFusion/CMakeLists.txt:48` compiles with
  `-gencode arch=compute_89,code=compute_89`, which produces only `compute_89` PTX and no machine code
  (SASS) for any other GPU.
- `cuobjdump --list-ptx` on `install/cuda_bevfusion_vendor/opt/cuda_bevfusion_vendor/lib/libbevfusion_core.so`
  confirms that it contains only `sm_89` PTX. That PTX can't be JIT-compiled for an 8.6 device, so the
  first kernel launch (`create_frustum_kernel`) fails and the process aborts.

## Fix

Rebuild the vendor package for compute capability 8.6. Clear its build directory first, because
`CUDASM=89` is saved in the generated ExternalProject build files:

```bash
cd /ws
rm -rf build/cuda_bevfusion_vendor install/cuda_bevfusion_vendor
export CUDASM=86
colcon build --packages-up-to bevfusion
```

To stop this happening again, do one of the following:

- Set `export CUDASM=86` in your shell profile, Dockerfile, or dev-container environment.
- Change the default in `cuda_bevfusion_vendor/CMakeLists.txt`, or better, detect it from
  `nvidia-smi --query-gpu=compute_cap`.
- Change the gencode flags to also emit SASS for several architectures, e.g.
  `-gencode arch=compute_86,code=sm_86 -gencode arch=compute_89,code=sm_89`.

## Other things noticed

1. **The TensorRT engines may also need rebuilding.** The `.plan` files in
   `/opt/watonomous/models/bevfusion/resnet50int8_trt11/build/` (dated Oct 23) are tied to the GPU
   they were built on. If they weren't built on this RTX 3090, delete that `build/` folder; the node
   will rebuild the engines on its first configure, which takes several minutes.

2. **Why Ctrl-C hung during shutdown:** `detection_lifecycle_manager` was still waiting on the
   configure call to the dead node (`transition_timeout_s: 130.0`), so it ignored SIGINT and SIGTERM
   and had to be killed with SIGKILL. This is a side effect of the crash, not a separate bug.

3. **Wrong default model path:** `bevfusion/config/params.yaml` and the built-in default in
   `bevfusion_node.cpp:80` point to `/opt/watonomous/models/bevfusion/resnet50_trt11_int8`, which
   doesn't exist. `perception_bringup.yaml` overrides it with the correct `resnet50int8_trt11`, but
   running the package's own `bevfusion.launch.yaml` would fail to find the models.

## How it was diagnosed

1. `log/latest_build/events.log`: every build job ended with `rc=0`.
2. `~/.ros/log/latest/launch.log`: the container died with exit code -6 right after configuring
   `bevfusion_node` began.
3. The container's log ended without an error, because the abort message went to the terminal's
   stderr.
4. Ran `ros2 run bevfusion bevfusion_node --ros-args --params-file <perception_bringup.yaml>` and
   `ros2 lifecycle set /bevfusion_node configure`, which reproduced the crash and captured the CUDA error.
5. `nvidia-smi -L` showed the RTX 3090. `cuobjdump` and the generated build files showed `CUDASM=89`.
