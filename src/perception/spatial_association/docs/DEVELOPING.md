# Spatial Association Node - Developer Guide

## Overview

The `spatial_association` node is a ROS2 perception node that performs 3D object detection by combining LiDAR point cloud data with 2D camera detections. It clusters LiDAR points, associates them with camera detections using IoU (Intersection over Union) matching, and applies quality filtering before outputting 3D boxes.

## Architecture

The node is structured in three main layers:

### 1. ROS2 Node Layer (`spatial_association`)
- **File**: `src/spatial_association.cpp`, `include/spatial_association/spatial_association.hpp`
- Handles ROS2 communication (subscribers, publishers, parameters)
- Coordinates between LiDAR processing and camera detection matching
- Manages TF transforms between sensor frames

### 2. Core Processing Layer (`SpatialAssociationCore`)
- **File**: `src/spatial_association_core.cpp`, `include/spatial_association/spatial_association_core.hpp`
- Performs point cloud downsampling (voxel grid)
- Executes clustering pipeline (Euclidean clustering → filtering → merging)
- Stateless, reusable library (no ROS dependencies)

### 3. Utility Layer (`ProjectionUtils`)
- **File**: `utils/src/projection_utils.cpp`, `utils/include/utils/projection_utils.hpp`
- Point cloud clustering algorithms
- Cluster quality filtering (configurable via `quality_filter_params`)
- 3D bounding box computation
- LiDAR-to-camera projection
- IoU computation for detection matching

## Data Flow

```
┌─────────────────┐
│ Non-ground Cloud│ (from patchwork)
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Voxel Downsample│
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Euclidean       │
│ Clustering      │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Cluster Merging │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Build Candidates│
└────────┬────────┘
         │
         ▼
┌─────────────────┐     ┌─────────────────┐
│ Camera          │     │ IoU Matching    │
│ Detections      │────▶│ & Filtering     │
└─────────────────┘     └────────┬────────┘
                                │
                                ▼
                        ┌─────────────────┐
                        │ Class-Aware     │
                        │ Filter          │
                        └────────┬────────┘
                                │
                                ▼
                        ┌─────────────────┐
                        │ 3D Bounding     │
                        │ Boxes &         │
                        │ Detections      │
                        └─────────────────┘
```

### Key Components

#### 1. Class-Aware Filtering

**`filterCandidatesByClassAwareConstraints()`** runs after IoU matching and applies size/quality constraints (same defaults for all candidates). Constraint values are configured via `quality_filter_params` in `params.yaml`.

#### 2. Multi-camera behavior

The node subscribes to **`deep_msgs::msg::MultiDetection2DArray`** (from deep_ros `deep_object_detection`), which contains one `Detection2DArray` per camera, each with its own `header.frame_id` and `detections[]`.

- **Per-camera processing**: For each camera in `camera_detections`, the node uses the **same** lidar cloud and a **copy** of the same cached cluster candidates. It runs the full pipeline (IoU assignment, class-aware filter, 3D box computation) independently for that camera.
- **Concatenation**: All 3D detections from all cameras are appended into a single `Detection3DArray` (`merged_detections3d`) and published on `/detection_3d`. Bounding box markers are namespaced by camera via `marker.ns = frame_id`.
- **No cross-camera deduplication**: If multiple cameras see the same physical object, the same lidar cluster can be assigned to a 2D detection in each camera. The result is **multiple 3D detections** for the same object (one per camera). There is no 3D NMS or merge step. Downstream (e.g. tracking) may perform its own merge or NMS if a single detection per object is required.
- **Frame and sync requirements**: Each camera’s `header.frame_id` must match an entry in `MultiCameraInfo`; detection–cloud stamp delta can be checked via `max_detection_cloud_stamp_delta_` (cameras exceeding it are skipped).

## Key Functions

### Clustering Pipeline

The pipeline uses a single container type **`ClusterCandidate`** (indices + stats + optional match) so alignment cannot go stale:

1. **Cluster** – `SpatialAssociationCore::performClustering()`: Euclidean clustering, then merge.
2. **Build candidates** – `ProjectionUtils::buildCandidates(cloud, cluster_indices)`: one struct per cluster with indices and stats.
3. **IoU match** – `ProjectionUtils::assignCandidatesToDetectionsByIOU(cloud, candidates, ...)`: fills `candidate.match`, removes unmatched.
4. **Class-aware filter** – `ProjectionUtils::filterCandidatesByClassAwareConstraints(candidates, detections)`: reads quality thresholds from `ProjectionUtils::getParams()` (filled from `quality_filter_params` in `initializeParams()` / `setParams`).
5. **Box fitting** – boxes from `extractIndices(candidates)`, then `compute3DDetection(boxes, candidates, ...)` for class/score from `candidate.match`.

#### `SpatialAssociationCore::performClustering()`

1. **Euclidean Clustering**: Groups nearby points (standard or adaptive).
2. **Cluster Merging**: `mergeClusters()` – merges nearby clusters. Physics filtering is done in the node on candidates.

#### `ProjectionUtils::computeClusterStats()` / `buildCandidates()`

Computes statistics for each cluster:
- Centroid (3D position)
- Bounding box (min/max x, y, z)
- Point count

Used throughout the filtering pipeline.

### Detection Matching

#### `ProjectionUtils::assignCandidatesToDetectionsByIOU()`

Greedy one-to-one assignment on `std::vector<ClusterCandidate>`: fills `candidate.match` for kept clusters, removes unmatched. Matches 3D LiDAR clusters with 2D camera detections:
1. For each cluster, projects the 8 AABB corners (from `ClusterStats`) to the image to build a 2D bounding rect
2. Falls back to centroid projection when all AABB corners are behind the camera
3. Computes IoU with each camera detection
4. Keeps only clusters with IoU ≥ `min_iou_threshold` (0.11)
5. Optional second pass (`enable_second_pass_fallback`): for *unassigned* detections, rescues candidates using stronger evidence than IoU-only:
   - centroid must land inside an expanded 2D detection bbox
   - projected point support inside the expanded bbox must pass `inside_count`/`inside_fraction` thresholds
   - projected footprint size plausibility must pass
   - the best match is chosen by a combined score `0.35*iou + 0.30*inside_fraction + 0.20*center_score + 0.15*size_score`
   - acceptance requires the best score to exceed `second_pass_min_combined_score` and be separated from the second-best by `second_pass_best_second_margin`

### Bounding Box Computation

#### `ProjectionUtils::computeClusterBoxes()`

Implementation is in `utils/cluster_box_utils.cpp` (`cluster_box` namespace): search-based xy orientation, percentile bounds, optional sigma clipping. `ProjectionUtils::computeClusterStats()` and merge logic call the same module for consistent bounds.

Computes oriented 3D bounding boxes:
- Uses search-based fitting for orientation
- Handles tall objects (cuts bottom portion for orientation)
- Returns `Box3D` with center, size, and yaw

## Configuration Parameters

### ROS2 Parameters (`config/params.yaml`)

#### Euclidean Clustering

```yaml
euclid_params:
  cluster_tolerance: 0.40        # Reference tolerance (m); each band uses cluster_tolerance * multiplier
  min_cluster_size: 20          # Minimum points per cluster
  max_cluster_size: 1200        # Maximum points per cluster
  use_adaptive_clustering: true # Radial bands with tighter tolerance near ego (better for 2D association)
  close_threshold: 10.0         # Upper radius (m) of near band
  mid_threshold: 25.0           # Upper radius of mid band (set ≤ close_threshold for two-band near/far only)
  close_tolerance_mult: 0.65    # Near-band multiplier (typically < 1)
  mid_tolerance_mult: 1.125     # Mid-band multiplier
  far_tolerance_mult: 1.75      # Far-band multiplier (typically > 1 for sparse returns)
```

#### Quality Filter Parameters
The `filterCandidatesByClassAwareConstraints()` function uses the `quality_*` fields in `ProjectionUtilsParams`, loaded from `quality_filter_params` in `params.yaml` alongside other `projection_utils_params` in `initializeParams()`.

#### Other Parameters

```yaml
merge_threshold: 0.1                     # Max AABB gap distance between clusters to merge (m)
object_detection_confidence: 0.52        # Min camera detection confidence
voxel_size: 0.2                         # Voxel grid leaf size (m)
# Visualization toggles (independent):
publish_bounding_box: true    # 3D bounding box marker visualization
publish_visualization: false # Debug point-cloud topics (filtered lidar, cluster centroids)
publish_image_visualization: false  # Image visualization (multi_image subscription for overlays; off by default)
debug_logging: false                     # Enable verbose logging
```

### Topic Configuration

#### Subscribers
- `multi_camera_info` (remapped to e.g. `/multi_camera_sync/multi_camera_info`): **Single batched** camera calibration from the deep_ros camera_sync node. The node does not subscribe to individual per-camera `camera_info` topics.
- `multi_image` (remapped to `/multi_camera_sync/multi_image_compressed`): Batched **compressed** images from camera_sync, for image-based visualizations. Subscribed only when `publish_image_visualization: true`.
- `non_ground_cloud`: Non-ground filtered point cloud (from patchwork++)
- `detections` (`deep_msgs::msg::MultiDetection2DArray`): Per-camera 2D detections from deep_object_detection; see **Multi-camera behavior** above.

#### Camera name / frame_id (deep_ros compatibility)
- The same **camera name** (frame_id) must be used in three places so the node can associate detections with camera info and TF:
  1. **MultiCameraInfo**: each `camera_infos[i].header.frame_id` (set by camera_sync from its `camera_names` or from the source CameraInfo).
  2. **Detection2DArray**: `header.frame_id` (set by deep_object_detection per batched image; must match the camera name for that image).
  3. **TF**: the camera optical frame name in the TF tree (published by sensor_interfacing) must match so `lookupTransform(camera_frame_id, lidar_frame)` succeeds.
- Entries in MultiCameraInfo with empty `header.frame_id` are skipped and a throttled warning is logged.

#### Transforms (TF)
- Camera↔lidar transforms are **not** hardcoded. The node uses the TF tree (subscribes via `tf2_ros::Buffer`/`TransformListener`) to look up the transform from the lidar frame to each camera frame. These transforms are expected to be published by **sensor_interfacing** (e.g. static transforms or from the robot description).
- Parameter `lidar_frame` (default: `lidar_cc`) specifies the frame of the lidar point cloud. Use `lidar_cc` when the non-ground cloud comes from the center lidar (e.g. `/lidar_cc/velodyne_points` → patchwork++ → `non_ground_cloud`).

#### Publishers
- `/detection_3d`: 3D detections (`vision_msgs::msg::Detection3DArray`)
- `/bounding_box`: 3D bounding box markers (if `publish_bounding_box: true`)
- `/filtered_lidar`: Colored cluster cloud (if `publish_visualization: true`)
- `/cluster_centroid`: Cluster centroids (if `publish_visualization: true`)

## Code Structure

```
spatial_association/
├── src/
│   ├── spatial_association.cpp          # ROS2 node implementation
│   └── spatial_association_core.cpp     # Core clustering logic
├── include/
│   └── spatial_association/
│       ├── spatial_association.hpp      # ROS2 node header
│       └── spatial_association_core.hpp # Core class header
├── utils/
│   ├── src/
│   │   └── projection_utils.cpp         # Utility functions + filtering
│   └── include/
│       └── utils/
│           └── projection_utils.hpp     # Utility header
├── config/
│   └── params.yaml                      # ROS2 parameters
└── launch/
    └── spatial_association_launch.yaml  # Launch file
```

## Key Algorithms

### 1. Adaptive Euclidean Clustering

Splits the cloud by radial distance from the lidar origin into up to three bands, then runs Euclidean clustering **separately** in each band with a band-specific tolerance `cluster_tolerance * multiplier`:

- **Near** (`[0, close_threshold)`): `close_tolerance_mult` — keep **below 1.0** so dense, nearby returns do not merge separate objects (helps one-to-one 2D association).
- **Mid** (`[close_threshold, mid_threshold)`): `mid_tolerance_mult` — omitted if `mid_threshold` ≤ `close_threshold`.
- **Far** (`[mid_threshold, ∞)` when mid is enabled, else `[close_threshold, ∞)`): `far_tolerance_mult` — often **> 1.0** to link sparser far-range points.

For ROI-centric association, consider `use_roi_first_clustering` in `params.yaml`.

### 2. Search-Based Orientation Fitting

For computing bounding box orientation:
1. **Coarse search**: Tests angles in 10° steps (0° to 90°)
2. **Fine search**: Refines best angle in ±5° range with 2° steps
3. **Edge energy minimization**: Finds orientation that minimizes distance to bounding box edges

### 3. Outlier Rejection

For small clusters (< 30 points):
- Computes mean and standard deviation of rotated coordinates
- Clips points beyond `4.5σ` to prevent outliers from skewing bounding box

### 4. Aspect Ratio Disambiguation

When aspect ratio < 1.2 (nearly square):
- Uses line-of-sight yaw (from centroid to sensor)
- Otherwise uses fitted orientation, choosing the direction closest to line-of-sight

## Debugging

### Enable Debug Logging

Set in `params.yaml`:

```yaml
debug_logging: true
```

This enables:
- Point cloud size logging
- Cluster count before/after filtering
- IoU matching statistics

### Visualization

Enable visualization topics:

```yaml
publish_visualization: true
```

Then visualize in RViz:
- `/bounding_box`: 3D bounding box markers
- `/filtered_lidar`: Colored point cloud (each cluster has random color)
- `/cluster_centroid`: Cluster centroids as points

## Performance Considerations

### Voxel Downsampling
- Default: `0.2m` voxel size
- Reduces point cloud size significantly
- Trade-off: Lower resolution vs. faster processing

### Cluster Size Limits
- `min_cluster_size: 20`: Prevents tiny noise clusters
- `max_cluster_size: 1200`: Prevents processing huge clusters (likely ground artifacts)

### Adaptive Clustering
- Reduces fragmentation of close objects
- Improves detection quality for nearby vehicles/pedestrians

## Common Issues & Solutions

### Issue: Too Many False Positives

**Solution**: Adjust filtering thresholds in `quality_filter_params` in `params.yaml`:
- Increase `min_density` (currently 5.0)
- Decrease `max_aspect_ratio` (currently 15.0)
- Adjust distance-adaptive point thresholds

### Issue: Missing Valid Objects

**Solution**: Relax filtering constraints:
- Decrease `min_density` in `quality_filter_params`
- Increase `max_aspect_ratio`
- Lower minimum point thresholds in distance-adaptive logic

### Issue: Clusters Too Fragmented

**Solution**: Increase `cluster_tolerance` and/or raise the **far** (and optionally **mid**) multipliers; avoid pushing `close_tolerance_mult` above 1.0 if nearby objects incorrectly merge.

```yaml
euclid_params:
  cluster_tolerance: 0.50
  use_adaptive_clustering: true
  far_tolerance_mult: 2.0
```

### Issue: TF Transform Errors

**Solution**: Ensure TF tree is properly configured:
- Check that `lidar_frame` parameter matches actual frame name
- Verify camera frames match camera info topics
- Use `ros2 run tf2_ros tf2_echo` to debug transforms

## Testing

### Unit Testing

Test individual components:

```cpp
// Test filtering (class-aware filter uses quality_filter_params from node/core)
std::vector<ProjectionUtils::ClusterCandidate> candidates = ...;
ProjectionUtils::filterCandidatesByClassAwareConstraints(candidates, detections);
```

### Integration Testing

1. Launch node with test data
2. Check output topics:

```bash
   ros2 topic echo /detection_3d
   ```

1. Verify cluster counts match expectations
2. Check for false positives/negatives

## References

- **PCL Documentation**: http://pointclouds.org/documentation/
- **2D-3D IOU Paper**: https://arxiv.org/pdf/1908.03851

---
