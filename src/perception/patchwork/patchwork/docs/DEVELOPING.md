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

Node type: **Lifecycle Node** (`rclcpp_lifecycle::LifecycleNode`)

The node implements the ROS 2 lifecycle pattern for controlled startup, shutdown, and reconfiguration. It can be managed via `lifecycle_manager` or run standalone (automatically configures and activates).

Topics (relative, remappable):

- Subscription
  - `input_cloud` (`sensor_msgs/msg/PointCloud2`)
- Publications
  - `ground_cloud` (`sensor_msgs/msg/PointCloud2`)
  - `non_ground_cloud` (`sensor_msgs/msg/PointCloud2`)

QoS:

- Configurable via parameters (see Parameters section below).
- Default subscriber: `best_effort` reliability for low-latency sensor data.
- Default publishers: `reliable` + `transient_local` durability for immediate availability to late joiners (e.g., RViz).

Launch options:

- Local launch YAML: `src/perception/patchwork/launch/ground_removal_launch.yaml:1` supports a `cloud_topic` argument.
- Perception bringup: `src/perception/perception_bringup/launch/perception.launch.py:81` composes this node with remappable input/output topics and parameter file.
- Component composition: The node is registered as a composable component via `RCLCPP_COMPONENTS_REGISTER_NODE` and can be loaded dynamically.

## Parameters

Parameters are declared in the node constructor and forwarded into `patchwork::Params` during `on_configure()`. Defaults are set in both the node and `config/params.yaml`. The canonical list lives in the node constructor and YAML.

### Patchwork++ Algorithm Parameters

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

### QoS Configuration Parameters

- `qos_subscriber_reliability` (string): Reliability policy for input subscriber. Options: `"reliable"`, `"best_effort"` (default: `"best_effort"`).
- `qos_subscriber_depth` (int): Queue depth for incoming messages (default: `10`).
- `qos_publisher_reliability` (string): Reliability policy for output publishers. Options: `"reliable"`, `"best_effort"` (default: `"reliable"`).
- `qos_publisher_durability` (string): Durability policy for output publishers. Options: `"transient_local"`, `"volatile"` (default: `"transient_local"`).
- `qos_publisher_depth` (int): Queue depth for outgoing messages (default: `10`).

### Point Cloud Size Validation

**Note**: Point cloud size validation has been removed from production code. Users are responsible for ensuring reasonable input sizes by running tests before deploying. The test suite includes validation tests for various point cloud sizes.

Edit defaults in `src/perception/patchwork/config/params.yaml:1` or pass overrides via launch.

## Control Flow & Data Model

### Lifecycle State Management

The node follows the ROS 2 lifecycle pattern:

1. **Unconfigured → Inactive** (`on_configure`): Validates parameters, initializes `GroundRemovalCore`, sets up QoS profiles, and initializes diagnostic updater. Does not create subscribers/publishers yet.
2. **Inactive → Active** (`on_activate`): Creates subscribers and publishers, activates lifecycle publishers, and sets up topic diagnostics. Node starts processing point clouds.
3. **Active → Inactive** (`on_deactivate`): Stops processing but keeps resources allocated for quick reactivation. Deactivates publishers and resets subscriber.
4. **Inactive → Unconfigured** (`on_cleanup`): Destroys subscribers, publishers, releases core resources, and cleans up diagnostics. Node can be reconfigured after cleanup.
5. **Any → Finalized** (`on_shutdown`): Performs final cleanup and resource release. Node cannot be restarted after shutdown.

### High‑level Processing Path (`removeGround`)

1. Receive `PointCloud2` on `input_cloud`.
2. Convert to `Eigen::MatrixX3f` (N×3) via `GroundRemovalCore::pointCloud2ToEigen`.
3. Validate point cloud dimensions against configured limits (`validatePointCloudDimensions`).
4. Filter out NaN/Inf values (`filterInvalidPoints`) — allows processing to continue with valid points even if some are invalid.
5. Call `GroundRemovalCore::process`, which forwards into `patchwork::PatchWorkpp::estimateGround`.
6. Fetch segmented sets: `getGround()` and `getNonground()` (both `Eigen::MatrixX3f`).
7. Convert each set back to `PointCloud2` via `GroundRemovalCore::eigenToPointCloud2`.
8. Publish to `ground_cloud` and `non_ground_cloud` with the original header (only if publishers are activated).
9. Update statistics (`updateStatistics`) and diagnostics (`updateDiagnostics`).
10. Emit a debug log with counts and processing time.

### Design Notes

- Data format is Nx3 single‑precision floats. Additional fields (intensity, ring, etc.) are not preserved in output — the publishers intentionally produce minimal XYZ‑only clouds to reduce message size and complexity.
- Endianness: conversions honor the message `is_bigendian` flag; writing outputs sets `is_bigendian=false`.
- The upstream library provides `getTimeTaken()` to report per‑frame processing time (in milliseconds).
- Point cloud filtering: The node filters invalid points (NaN/Inf) before processing. Size validation is not performed in production code - users should test their inputs.
- Statistics tracking: Processing statistics (total processed, average time, last processing time) are tracked atomically and logged periodically (every 30 seconds by default).

## Conversion Utilities (PointCloud2 ↔ Eigen)

Defined in `src/perception/patchwork/src/ground_removal_core.cpp:1` and declared in `src/perception/patchwork/include/patchworkpp/ground_removal_core.hpp:1`.

### `pointCloud2ToEigen(PointCloud2)`

- Validates presence of `x`, `y`, `z` fields and that each is `FLOAT32`.
- Validates point cloud dimensions (width, height, total points) against reasonable limits to prevent memory exhaustion.
- Supports arbitrary `height`, `width`, `point_step`, `row_step`, and endianness.
- Returns a contiguous `Eigen::MatrixX3f` with one point per row.
- Throws `std::runtime_error` if required fields are missing, of wrong type, or dimensions exceed limits.

### `eigenToPointCloud2(Eigen::MatrixX3f, Header)`

- Produces a minimal XYZ‑only `PointCloud2` with fields at offsets 0, 4, and 8 (FLOAT32), `height=1`, `width=N`, `is_bigendian=false`, `is_dense=false`.
- Copies the provided header (frame id and stamp preserved).

### Helper Functions

- `findFieldIndex(fields, field_name)`: Static private helper to locate field indices by name. Returns -1 if not found.
- Internal helpers (`readFloat`, `writeFloat`, `validateDimensions`) are in an anonymous namespace to prevent name pollution.

Pattern: avoid PCL dependency for simple conversions — keep a small, explicit converter to minimize footprint and control performance.

## CMake & Linking Patterns

See `src/perception/patchwork/CMakeLists.txt:1`.

- Use C++17 with `-Wall -Wextra -Wpedantic` on GCC/Clang.
- Find Patchwork++ via `find_package(patchworkpp REQUIRED)` and normalize the target to `PATCHWORKPP_TARGET` by checking the exported alias names.
- Link against `Eigen3::Eigen` and ROS 2 components explicitly.
- Install the node binary and the `launch/` and `config/` directories.

If Patchwork++ changes its exported target names again, extend the conditional target check in CMake accordingly.

## Logging, QoS, Diagnostics, and Executors

### Logging

- Initialization logs at INFO describe subscriptions and publications.
- Per‑message logs are at DEBUG level to avoid log spam under load (enable with `RCLCPP_LOG_LEVEL=DEBUG`).
- Error and warning messages include actual values vs limits for better debugging (e.g., "received 12000000 points, but maximum allowed is 10000000 points").
- Statistics are logged periodically (every 30 seconds by default) with total processed clouds and average processing time.

### QoS

- QoS settings are configurable via parameters (see Parameters section).
- Default subscriber: `best_effort` reliability for low-latency sensor data.
- Default publishers: `reliable` + `transient_local` durability for RViz convenience; change to volatile if transient behavior is undesired in production.

### Diagnostics

- The node integrates with `diagnostic_updater` for health monitoring.
- Topic diagnostics track publishing frequency and timestamp validity for both ground and non-ground publishers.
- Custom diagnostic callback reports:
  - Total clouds processed
  - Average processing time (ms)
  - Last processing time (ms)
  - Status (OK/WARN) based on latency thresholds (avg >100ms or last >200ms triggers WARN)

### Executors

- The `main` uses a `MultiThreadedExecutor`, but this node has a single subscription callback — no shared mutable state across callbacks beyond the core instance, so there is no concurrent mutation risk under the default configuration.
- Statistics tracking uses `std::atomic` for thread-safe updates.

## Performance Considerations

- The converters allocate per message; for very high throughput, consider pre‑allocating buffers and reusing them (requires tracking capacity and careful lifetime management).
- Keep `num_iter` moderate; higher values increase compute time non‑linearly.
- Validate `sensor_height` and ranges to reduce misclassifications that can cause downstream churn.
- Prefer Release builds for runtime deployments (`-DCMAKE_BUILD_TYPE=Release`).

## Extending the Node/Core

### Adding a new Patchwork++ parameter:

1. Add a `declare_parameter` line in the node constructor.
2. Retrieve the parameter in `declareParameters()` and write into `patchwork::Params`.
3. Introduce the parameter in `config/params.yaml` with a sensible default.
4. Document the parameter here and in `README.md` if externally relevant.

### Publishing additional outputs (e.g., plane coefficients, masks, diagnostics):

1. Expose the data in `GroundRemovalCore` (new getters populated from the underlying Patchwork++ instance).
2. Add new lifecycle publishers in `on_activate()` and activate them.
3. Publish in `publishSegments()` or a dedicated method.
4. Consider QoS: diagnostics typically do not need transient local.

### Changing topics or QoS:

- Prefer remapping topics in launch files rather than changing constants in code.
- Adjust QoS via parameters (see Parameters section) or in the node constructor to keep behavior consistent across publishers.

### Component composition:

- `GroundRemovalNode` is registered as a composable component via `RCLCPP_COMPONENTS_REGISTER_NODE` and can be loaded dynamically.
- The node takes `rclcpp::NodeOptions` and supports both standalone execution and component composition.

### Code Organization Patterns:

- **Separation of Concerns**: Validation logic is in dedicated functions (`validatePointCloudDimensions`, `filterInvalidPoints`).
- **Helper Methods**: Statistics and diagnostics updates are in dedicated methods (`updateStatistics`, `updateDiagnostics`).
- **Anonymous Namespaces**: Internal helpers (e.g., in `ground_removal_core.cpp`) use anonymous namespaces to prevent name pollution.
- **Documentation**: All docstrings are in header files; implementation files are kept minimal with self-documenting code.

## Testing & Validation

The package includes comprehensive test suites covering core functionality and ROS integration.

### Running Tests

#### Build Tests

First, ensure tests are built:

```bash
colcon build --packages-select patchworkpp
```

#### Run All Tests

Run all tests for the package:

```bash
colcon test --packages-select patchworkpp
colcon test-result --verbose
```

#### Run Specific Test Suites

**Core/LiDAR Processing Tests** (`test_ground_removal_core`):
- Tests point cloud conversion, validation logic, and algorithm processing
- No ROS dependencies required

```bash
colcon test --packages-select patchworkpp --ctest-args -R test_ground_removal_core
colcon test-result --verbose
```

**ROS Integration Tests** (`test_ground_removal_node`):
- Tests lifecycle node behavior, QoS configuration, callbacks, statistics, and parameters
- Requires ROS 2 environment

```bash
colcon test --packages-select patchworkpp --ctest-args -R test_ground_removal_node
colcon test-result --verbose
```

#### Run Tests by Tag

Tests are organized with Catch2 tags for selective execution:

**Core conversion tests:**
```bash
colcon test --packages-select patchworkpp --ctest-args -R test_ground_removal_core -E "[ground_removal_core]"
```

**Validation logic tests:**
```bash
colcon test --packages-select patchworkpp --ctest-args -R test_ground_removal_core -E "[validation]"
```

**ROS integration tests:**
```bash
colcon test --packages-select patchworkpp --ctest-args -R test_ground_removal_node -E "[ros_integration]"
```

**Statistics and diagnostics tests:**
```bash
colcon test --packages-select patchworkpp --ctest-args -R test_ground_removal_node -E "[statistics]"
```

**Parameter handling tests:**
```bash
colcon test --packages-select patchworkpp --ctest-args -R test_ground_removal_node -E "[parameters]"
```

#### Run Tests Directly (for debugging)

To run a specific test executable directly with verbose output:

```bash
# Core tests
/home/bolty/ament_ws/build/patchworkpp/test_ground_removal_core -s "GroundRemovalCore Conversion Tests"

# Node tests (requires ROS environment)
source /opt/ros/jazzy/setup.bash
source /home/bolty/ament_ws/install/setup.bash
/home/bolty/ament_ws/build/patchworkpp/test_ground_removal_node -s "ROS Integration Tests"
```

#### Re-run Failed Tests

To re-run only failed tests with verbose output:

```bash
colcon test --packages-select patchworkpp --ctest-args --rerun-failed --output-on-failure
colcon test-result --verbose
```

### Test Coverage

#### Core Tests (`test_ground_removal_core.cpp`)

**PointCloud2 ↔ Eigen Conversion Tests:**
- Valid data conversion
- NaN/Inf value handling
- Empty point cloud handling
- Missing field error handling
- Large dimension handling (no validation limits)

**Point Cloud Validation Logic Tests:**
- Size limits (empty, single point, small, medium, large, very large)
- NaN/Inf filtering behavior
- Edge cases (all points invalid)

**GroundRemovalCore Processing Tests:**
- Smoke test with synthetic plane data
- Robustness to different parameter configurations
- Eigen to PointCloud2 conversion

#### ROS Integration Tests (`test_ground_removal_node.cpp`)

**ROS Integration Tests:**
- Topic name constants verification
- Lifecycle state transitions (configure, activate, deactivate)
- QoS configuration (subscriber/publisher reliability, durability)
- Publisher activation state checking
- `removeGround()` callback message handling
- Empty point cloud handling
- Invalid point cloud error handling

**Statistics and Diagnostics Tests:**
- Statistics logging interval constant
- Statistics update (counter increments)
- Diagnostic callback status levels

**Parameter Handling Tests:**
- Parameter declaration and retrieval
- Parameter default values verification
- Invalid parameter values (unknown QoS strings)
- Parameter modification and reconfiguration

### Manual Validation

For manual testing with real data:

- Launch with a recorded bag or live sensor and view outputs in RViz.
- Confirm frame IDs and timestamps are preserved end‑to‑end.
- Inspect topic stats for throughput and latency; correlate with the core's `getTimeTaken()` debug log for processing time.

### Test Requirements

- **Core tests**: No special requirements, can run in any environment
- **Node tests**: Require ROS 2 environment (automatically set up in Docker container)
- **Dependencies**: `wato_test` package must be built before running node tests

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
- **Header/Implementation Separation**: All method declarations in headers with comprehensive Doxygen docstrings; implementations in `.cpp` files with minimal comments.
- **Anonymous Namespaces**: Use anonymous namespaces for internal helper functions to prevent name pollution.
- **Error Messages**: Include actual values vs limits in error messages for better debugging (e.g., "received X points, but maximum allowed is Y points").
- **Lifecycle Pattern**: Follow ROS 2 lifecycle node conventions — configure in `on_configure()`, activate in `on_activate()`, clean up resources properly.
- **Atomic Operations**: Use `std::atomic` for thread-safe statistics tracking.

## Known Limitations

- Output clouds do not retain input fields other than XYZ.
- The node does not currently expose plane model coefficients; add this via the extension pattern above if needed.
- The package assumes a single LiDAR input; multi‑sensor fusion is out of scope here.
- Point cloud size validation is not performed in production code. Users should run tests to ensure their point cloud inputs are reasonable for their use case.
- Invalid points (NaN/Inf) are filtered out rather than causing the entire cloud to be rejected; this allows processing to continue with valid points.

## Maintenance Tips

- If upstream Patchwork++ changes its public API, encapsulate those changes within `GroundRemovalCore` so the ROS 2 node interface remains stable.
- Keep parameter names stable; if renaming is required, provide deprecation windows and alias support in `declareParameters`.
- Validate external changes by running with representative datasets (e.g., nuScenes, custom platform logs) before merging.
