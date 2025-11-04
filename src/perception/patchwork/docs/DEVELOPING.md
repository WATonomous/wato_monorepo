# Patchwork++ Ground Removal — Developer Guide

This document provides technical specifications and implementation patterns for the Patchwork++ ROS 2 wrapper in this repository. It explains architecture, runtime interfaces, parameterization, build/dependency details, and patterns to follow when extending or maintaining the code.

## Overview

Patchwork++ is a LiDAR ground segmentation algorithm. This package wraps the upstream Patchwork++ C++ library in a ROS 2 node that:

- Subscribes to a raw `sensor_msgs/PointCloud2` stream.
- Segments points into ground and non‑ground using Patchwork++.
- Publishes the two segmented clouds for downstream consumers and visualization.

Key sources:

- CMake configuration: `src/perception/patchwork/CMakeLists.txt:1`
- Core (library-facing) API: `src/perception/patchwork/include/patchworkpp/ground_removal_core.hpp:1`
- Node (ROS-facing) API: `src/perception/patchwork/include/patchworkpp/ground_removal_node.hpp:1`
- Core implementation: `src/perception/patchwork/src/ground_removal_core.cpp:1`
- Node implementation: `src/perception/patchwork/src/ground_removal_node.cpp:1`
- Launch (package local): `src/perception/patchwork/launch/ground_removal_launch.yaml:1`
- Default parameters: `src/perception/patchwork/config/params.yaml:1`

Directory layout:

- `include/patchworkpp/` — Public headers for the node and core (available to external code).
- `src/` — Implementations of the node and core.
- `config/params.yaml` — Default ROS 2 parameters for Patchwork++.
- `launch/` — Launch file for convenience.

## Build & Dependencies

This package depends on the upstream Patchwork++ library and common ROS 2 dependencies. The Docker build pulls and installs Patchwork++ into `/usr/local`:

- See `docker/perception/perception.Dockerfile:49` for the build and install of the third‑party library.
- The CMake script finds and links Patchwork++ under whichever target name is exported by the install (the upstream project has used multiple target names over time). We normalize to a single variable `PATCHWORKPP_TARGET`:

  - `patchworkpp::ground_seg_cores` (preferred if present)
  - `patchworkpp::patchworkpp` (fallback)

Other dependencies:

- ROS 2: `rclcpp`, `sensor_msgs`, `std_msgs`, `rcutils`, `geometry_msgs`, `nav_msgs`, `tf2_ros`
- Eigen: `eigen3_cmake_module`, `Eigen3`

Build with colcon (inside the container or a ROS 2 environment):

```
colcon build --packages-select patchworkpp --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## Runtime Interfaces

Node name: `patchworkpp_node`

Topics (relative, remappable):

- Subscription
  - `input_cloud` (`sensor_msgs/msg/PointCloud2`)
- Publications
  - `ground_cloud` (`sensor_msgs/msg/PointCloud2`)
  - `non_ground_cloud` (`sensor_msgs/msg/PointCloud2`)

QoS:

- Subscriber uses `rclcpp::SensorDataQoS()` for robust sensor data ingestion.
- Publishers use reliable + transient local QoS so the latest segmentation is latched for late joiners (e.g., RViz).

Launch options:

- Local launch YAML: `src/perception/patchwork/launch/ground_removal_launch.yaml:1` supports a `cloud_topic` argument.
- Perception bringup: `src/perception/perception_bringup/launch/perception.launch.py:81` composes this node with remappable input/output topics and parameter file.

## Parameters

Parameters are declared in the node and forwarded into `patchwork::Params`. Defaults are set in both the node and `config/params.yaml`. The canonical list lives in the node (`declareParameters`) and YAML.

Common parameters and intent (tune per platform/dataset):

- `sensor_height` (double): LiDAR height above ground in meters.
- `num_iter` (int): Iterations for plane refinement.
- `num_lpr` (int): Lowest points per ring for seed plane estimation.
- `num_min_pts` (int): Minimum points to process in a bin/sector.
- `th_seeds` (double): Seed selection vertical threshold.
- `th_dist` (double): Distance threshold for ground classification.
- `th_seeds_v` (double): Variation‑aware seed angular threshold.
- `th_dist_v` (double): Variation‑aware distance threshold.
- `max_range` (double): Maximum radial range to consider (meters).
- `min_range` (double): Minimum radial range to consider (meters).
- `uprightness_thr` (double): Max deviation of ground normal from vertical.
- `enable_RNR` (bool): Enable ring‑wise noise removal.
- `verbose` (bool): Enable verbose logging inside Patchwork++.

Edit defaults in `src/perception/patchwork/config/params.yaml:1` or pass overrides via launch.

## Control Flow & Data Model

High‑level processing path (`removeGround`):

1. Receive `PointCloud2` on `input_cloud`.
2. Convert to `Eigen::MatrixX3f` (N×3) via `GroundRemovalCore::pointCloud2ToEigen`.
3. Call `GroundRemovalCore::process`, which forwards into `patchwork::PatchWorkpp::estimateGround`.
4. Fetch segmented sets: `getGround()` and `getNonground()` (both `Eigen::MatrixX3f`).
5. Convert each set back to `PointCloud2` via `GroundRemovalCore::eigenToPointCloud2`.
6. Publish to `ground_cloud` and `non_ground_cloud` with the original header.
7. Emit a debug log with counts and processing time.

Design notes:

- Data format is Nx3 single‑precision floats. Additional fields (intensity, ring, etc.) are not preserved in output — the publishers intentionally produce minimal XYZ‑only clouds to reduce message size and complexity.
- Endianness: conversions honor the message `is_bigendian` flag; writing outputs sets `is_bigendian=false`.
- The upstream library provides `getTimeTaken()` to report per‑frame processing time (in milliseconds).

## Conversion Utilities (PointCloud2 ↔ Eigen)

Defined in `src/perception/patchwork/src/ground_removal_core.cpp:1` and declared in `src/perception/patchwork/include/patchworkpp/ground_removal_core.hpp:1`.

`pointCloud2ToEigen(PointCloud2)`:

- Validates presence of `x`, `y`, `z` fields and that each is `FLOAT32`.
- Supports arbitrary `height`, `width`, `point_step`, `row_step`, and endianness.
- Returns a contiguous `Eigen::MatrixX3f` with one point per row.
- Throws `std::runtime_error` if required fields are missing or of wrong type.

`eigenToPointCloud2(Eigen::MatrixX3f, Header)`:

- Produces a minimal XYZ‑only `PointCloud2` with fields at offsets 0, 4, and 8 (FLOAT32), `height=1`, `width=N`, `is_bigendian=false`, `is_dense=false`.
- Copies the provided header (frame id and stamp preserved).

Pattern: avoid PCL dependency for simple conversions — keep a small, explicit converter to minimize footprint and control performance.

## CMake & Linking Patterns

See `src/perception/patchwork/CMakeLists.txt:1`.

- Use C++17 with `-Wall -Wextra -Wpedantic` on GCC/Clang.
- Find Patchwork++ via `find_package(patchworkpp REQUIRED)` and normalize the target to `PATCHWORKPP_TARGET` by checking the exported alias names.
- Link against `Eigen3::Eigen` and ROS 2 components explicitly.
- Install the node binary and the `launch/` and `config/` directories.

If Patchwork++ changes its exported target names again, extend the conditional target check in CMake accordingly.

## Logging, QoS, and Executors

- Initialization logs at INFO describe subscriptions and publications.
- Per‑message logs are at DEBUG level to avoid log spam under load (enable with `RCLCPP_LOG_LEVEL=DEBUG`).
- Publishers are reliable + transient local for RViz convenience; change to volatile if transient behavior is undesired in production.
- The `main` uses a `MultiThreadedExecutor`, but this node has a single subscription callback — no shared mutable state across callbacks beyond the core instance, so there is no concurrent mutation risk under the default configuration.

## Performance Considerations

- The converters allocate per message; for very high throughput, consider pre‑allocating buffers and reusing them (requires tracking capacity and careful lifetime management).
- Keep `num_iter` moderate; higher values increase compute time non‑linearly.
- Validate `sensor_height` and ranges to reduce misclassifications that can cause downstream churn.
- Prefer Release builds for runtime deployments (`-DCMAKE_BUILD_TYPE=Release`).

## Extending the Node/Core

Adding a new Patchwork++ parameter:

1. Add a `declare_parameter` line in `declareParameters` (node) and write into `patchwork::Params`.
2. Introduce the parameter in `config/params.yaml` with a sensible default.
3. Document the parameter here and in `README.md` if externally relevant.

Publishing additional outputs (e.g., plane coefficients, masks, diagnostics):

1. Expose the data in `GroundRemovalCore` (new getters populated from the underlying Patchwork++ instance).
2. Add new publishers in the node and serialize the data to appropriate message types.
3. Consider QoS: diagnostics typically do not need transient local.

Changing topics or QoS:

- Prefer remapping topics in launch files rather than changing constants in code.
- Adjust QoS in one place (the node constructor) to keep behavior consistent across publishers.

Component composition:

- `GroundRemovalNode` takes `rclcpp::NodeOptions` and can be converted to a composable component (register via `rclcpp_components`) if needed.

## Testing & Validation

Manual checks:

- Launch with a recorded bag or live sensor and view outputs in RViz.
- Confirm frame IDs and timestamps are preserved end‑to‑end.
- Inspect topic stats for throughput and latency; correlate with the core’s `getTimeTaken()` debug log for processing time.

Suggested automated tests (not yet in repo):

- Unit tests for `pointCloud2ToEigen`/`eigenToPointCloud2` to cover field order, row/point steps, and endianness.
- Scenario tests with synthetic planes plus obstacles to verify split counts under known thresholds.

## Running

Using this repo’s bringup:

```
ros2 launch perception_bringup perception.launch.py \
  patchwork_cloud_topic:=/LIDAR_TOP \
  patchwork_ground_topic:=/patchworkpp/ground_cloud \
  patchwork_non_ground_topic:=/patchworkpp/non_ground_cloud
```

Package‑local launch:

```
ros2 launch patchworkpp ground_removal_launch.yaml cloud_topic:=/LIDAR_TOP
```

Docker (builds Patchwork++ and this package): see `docker/perception/perception.Dockerfile:1`. Inside the container, build and launch as above.

## Code Style & Conventions

- Namespace: `wato::perception::patchworkpp` for all code in this package.
- Use `Eigen::MatrixX3f` for point sets (N×3 layout, row‑major indexing style in code).
- Do not depend on PCL in this package; keep ROS↔Eigen conversions local and explicit.
- Preserve message headers when republishing derived data.
- Keep subscribers lightweight — heavy logic remains in `GroundRemovalCore`.

## Known Limitations

- Output clouds do not retain input fields other than XYZ.
- The node does not currently expose plane model coefficients; add this via the extension pattern above if needed.
- The package assumes a single LiDAR input; multi‑sensor fusion is out of scope here.

## Maintenance Tips

- If upstream Patchwork++ changes its public API, encapsulate those changes within `GroundRemovalCore` so the ROS 2 node interface remains stable.
- Keep parameter names stable; if renaming is required, provide deprecation windows and alias support in `declareParameters`.
- Validate external changes by running with representative datasets (e.g., nuScenes, custom platform logs) before merging.
