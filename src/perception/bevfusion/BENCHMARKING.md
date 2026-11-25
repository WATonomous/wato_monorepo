# LiDAR CPU/CUDA comparison

The benchmark keeps the model, camera decoding, camera synchronization, QoS, and
detection settings the same while selecting `cpu` or strict `cuda` lidar processing.
It does not use `auto`, so initialization failure cannot silently change the backend.
Timing output is disabled in normal operation (`benchmark_csv_path` defaults to empty).

## Inputs and preparation

Use `pano_lidar_cc_20261114_133353` from the
[perception bags folder](https://drive.google.com/drive/folders/1GzV_lhd0aW3-2a0DJo-mfSsaaLpOXWBR).
It contains 271 seconds of data, 5381 lidar scans, all six panoramic cameras at
approximately 20 Hz, camera calibration, and TF. Keep `metadata.yaml` and its MCAP
file together. Store downloads and outputs under `/tmp`, outside the repository;
temporary storage can disappear when the Watcloud machine/session is replaced.

Use the six-camera
[resnet50int8_trt11 deployment files](https://drive.google.com/drive/folders/1kmWQR3vaE1uWyqOW1u9wvxAQpqy-w7KR)
from the supplied weights folder. The required source files are
`camera.backbone.onnx`, `camera.vtransform.onnx`, `fuser.onnx`, `head.bbox.onnx`, and
`lidar.backbone.xyz.onnx`. The training checkpoint and `.bak` files are unnecessary.
The eight-camera and dual-head models require different input/model settings.

Build the matching NITROS, CUDA-BEVFusion, and monorepo packages and source their
overlays. Include `camera_sync` from the perception image's `deep_ros` checkout.
Set `CUDASM=86` when building on an RTX 3090; the vendor default of 89 targets newer
hardware. TensorRT plans must also be compatible with the installed TensorRT version
and GPU. Engine files alone do not establish that compatibility.

Run in a separate ROS domain on an idle GPU. The commands below execute **inside**
the prepared ROS/CUDA container; `/benchmark` and `/bags` are temporary host mounts.

```bash
source /benchmark/env.sh
ros2 run bevfusion benchmark_lidar check \
  --bag /bags/pano_lidar_cc_20261114_133353 \
  --model-dir /benchmark/models/resnet50_trt11_int8
```

`check` reads metadata and samples the input messages, including JPEG dimensions
and XYZI field types. It does not publish data or load a GPU model. It reports a
failure if required deployment engines are missing. Sampling TF confirms recorded
edges are present; successful calibration during a real trial is still required.

When the GPU is available, explicitly generate/load the four TensorRT engines:

```bash
ros2 run bevfusion benchmark_lidar prepare-models \
  --bag /bags/pano_lidar_cc_20261114_133353 \
  --model-dir /benchmark/models/resnet50_trt11_int8 \
  --output /benchmark/engine-preparation
```

This initializes BEVFusion and stops after activation, with a 15-minute startup
timeout. It uses the node's existing engine builder and writes plans into the model
directory's `build/` subdirectory. It does not replay the bag. Check `prepare.log`
if engine creation/loading fails. The comparison runner requires these plans to
exist before starting, keeping engine building outside measured trials.

## Run the comparison when ready

First run a short trial using the same command below with `--warmup 5 --duration 10
--repeats 1` and a separate output directory. Verify calibration succeeds and both
backends produce measurements. Then collect the normal comparison:

```bash
ros2 run bevfusion benchmark_lidar run \
  --bag /bags/pano_lidar_cc_20261114_133353 \
  --model-dir /benchmark/models/resnet50_trt11_int8 \
  --output /benchmark/results/baseline-1x \
  --warmup 30 --duration 180 --repeats 3 --rate 1
```

Each trial starts fresh camera-sync and BEVFusion processes, waits for activation,
replays only the required sensor/TF topics with simulated time, and allows five
seconds for draining. Trial pairs alternate order: CPU/CUDA, CUDA/CPU, CPU/CUDA.
Only these processes are stopped on completion or interruption. Output directories
must be new to avoid overwriting results. Six trials take approximately 21 minutes
plus initialization and cleanup. Rates above one shorten the replay wall time.

The launch uses the recorded compressed-image path, with NITROS cameras disabled
for both lidar backends. It retains the existing 500 ms camera batching tolerance
and the configured BEVFusion camera/lidar tolerance (normally 200 ms). It launches
no live sensor drivers. Reliable playback is compatible with both the camera-info
and lidar subscribers; `/tf_static` uses transient-local durability.

For increased-load comparisons, repeat with new output directories and `--rate 1.5`
or `--rate 2`. Run a separate short copy/kernel trace with `--nsys --warmup 5
--duration 10 --repeats 1`. Inspect the resulting `.nsys-rep` in Nsight Systems for
`NitrosPointCloud::convert_to_custom`, H2D transfers, filtering/scan/packing kernels,
and the lack of a full-cloud D2H transfer or a second lidar H2D upload. Distinguish
lidar transfers from the six camera uploads, count transfers per scan, and account
for scalar point-count downloads. Do not use profiled trials for final throughput.

## Read the results

Every trial saves:

- `timings.csv`: sensor timestamps, all six matched camera timestamps, input/valid
  point counts, processing stages, and node counters for each completed frame.
- `timings.csv.receipts.csv`: scans accepted by the selected subscriber, with
  monotonic timestamps. The CUDA timestamps are taken after adapter upload.
- `resources.csv`: cumulative CPU time and summed RSS for the ROS launch process
  tree and for the player separately, sampled once per second.
- `gpu.csv`: GPU utilization, memory, and power sampled once per second. These are
  device-wide measurements, so competing GPU work invalidates attribution.
- `detections.jsonl`: full output messages and observer timestamps for later
  numerical comparison of bounding boxes and confidence scores.
- `commands.json`, `qos.yaml`, and logs: the exact commands, replay QoS, and failures.

`manifest.json` saves input inspection, model directory, GPU/TensorRT information,
the node configuration, and trial parameters. The report excludes warmup using the
same lidar sensor-time interval for every trial and saves `summary.json`. Regenerate it with:

```bash
ros2 run bevfusion benchmark_lidar report /benchmark/results/baseline-1x
```

Compare per-trial p50/p95/p99 timings, output rates, received scans without completed
outputs, CPU percent (100% means one core), and memory. Check whether the requested
window has enough completed samples before interpreting its percentiles. Receipt
counts cannot detect DDS loss before the subscriber; bag counts describe the full
recording. A source-window rate includes completed outputs during the drain, while
the completed-span rate uses elapsed completion timestamps.

`lidar_ms` covers CPU extraction/filtering or GPU preprocessing, including the GPU
scalar-count synchronization. `cpu_fp16_ms` separately measures the CPU FP16
conversion and is a **subset** of `infer_ms`. `infer_ms` also includes host staging
and lidar upload on the CPU backend, camera/model work, and result conversion.
These are monotonic wall times, not separate CUDA kernel timings.

`ready_to_publish_ms` includes the wait for camera matching, image decoding,
preprocessing, inference, and publication after the subscriber has obtained the
cloud. **It excludes NITROS adapter upload**, which occurs before its callback.
Use the CUDA/NVTX trace to include that cost in the assessment. Buffered CSV writes
and diagnostics are outside the recorded stages but still affect throughput and
resource usage. Keep collection settings identical between backends.

The report counts identical `(lidar timestamp, six camera timestamps)` input sets
between paired runs. Use those sets to join output messages via `camera_stamp_ns`
and the detection header. Compare prepared-point counts, classes, box positions,
sizes, orientations, and scores, using repeated CPU trials to establish numerical
variation. The report does not automatically certify detection equivalence.
Lower preprocessing time alone does not establish higher whole-pipeline throughput.
