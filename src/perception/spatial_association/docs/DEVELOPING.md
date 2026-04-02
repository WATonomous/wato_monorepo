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
- Executes clustering pipeline (Euclidean clustering -> filtering -> merging)
- Stateless, reusable library (no ROS dependencies)

### 3. Utility Layer (`ProjectionUtils`)
- **File**: `utils/src/projection_utils.cpp`, `utils/include/utils/projection_utils.hpp`
- LiDAR-to-camera projection and IoU computation
- Multi-gate matching and scoring (centroid, inside fraction, aspect ratio)
- Hungarian algorithm integration for optimal assignment
- Class-aware quality filtering (size, density, range, height)
- 3D box fitting with orientation search or L-shaped fitting
- Detection array building and marker visualization

### Additional Utility Components
- **`cluster_box_utils`** (`utils/src/cluster_box_utils.cpp`): Oriented 3D box fitting with percentile-based bounds, L-shaped fitting for vehicles, class-aware box extension, outlier rejection via sigma clipping.
- **`hungarian`** (`utils/src/hungarian.cpp`): Hungarian/Munkres algorithm for optimal linear assignment.

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
│ (multi-band     │
│  adaptive)      │
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
│ Detections      │────>│ (strict +       │
└─────────────────┘     │  Hungarian)     │
                        └────────┬────────┘
                                │
                                ▼
                        ┌─────────────────┐
                        │ Class-Aware     │
                        │ Quality Filter  │
                        └────────┬────────┘
                                │
                                ▼
                        ┌─────────────────┐
                        │ 3D Box Fitting  │
                        │ (L-shaped /     │
                        │  search-based)  │
                        └────────┬────────┘
                                │
                                ▼
                        ┌─────────────────┐
                        │ Cross-Camera    │
                        │ Deduplication   │
                        └────────┬────────┘
                                │
                                ▼
                        ┌─────────────────┐
                        │ Detection3DArray│
                        │ + BBox Markers  │
                        └─────────────────┘
```

## Configuration Parameters

All parameters are in `config/params.yaml`.

### Node Identity & Timing

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `lidar_frame` | string | `lidar_cc` | LiDAR frame ID for TF lookups |
| `max_detection_cloud_stamp_delta` | double (s) | 0.08 | Max timestamp delta between detection and cloud; <=0 disables rejection |
| `detection_stamp_fallback_policy` | string | `median_camera_stamp` | Fallback for zero timestamp: `median_camera_stamp` or `max_camera_stamp` |
| `clustered_cloud_cache_size` | int | 24 | Ring buffer size for recent clustered snapshots |
| `raw_cloud_pending_queue_size` | int | 1 | Max raw clouds queued when `raw_cloud_latest_only` is false |
| `raw_cloud_latest_only` | bool | true | Each new cloud replaces entire pending queue |
| `skip_repeat_association_publish` | bool | true | Suppress repeated timestamp-pair associations |

### Clustering

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `voxel_size` | float | 0.2 | Voxel grid leaf size (m) |
| `euclid_params.cluster_tolerance` | double (m) | 0.40 | Base tolerance for Euclidean clustering |
| `euclid_params.min_cluster_size` | int | 20 | Minimum points per cluster |
| `euclid_params.max_cluster_size` | int | 1000 | Maximum points per cluster |
| `euclid_params.use_adaptive_clustering` | bool | true | Enable distance-adaptive tolerance |
| `euclid_params.close_threshold` | double (m) | 5.0 | Legacy close/far split threshold (ignored when `clustering_bands` is non-empty) |
| `euclid_params.close_tolerance_mult` | double | 1.25 | Legacy multiplier for close objects |
| `euclid_params.clustering_bands.max_distances` | array | [8.0, 20.0, 40.0, 999.0] | Multi-band distance boundaries (overrides legacy if non-empty) |
| `euclid_params.clustering_bands.tolerance_mults` | array | [1.3, 1.0, 1.5, 2.0] | Tolerance multiplier per band |
| `euclid_params.clustering_bands.min_cluster_sizes` | array | [15, 12, 10, 8] | Min cluster size per band |
| `merge_threshold` | double (m) | 0.4 | Max AABB gap distance between clusters to merge |

### 2D Detection Filter

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `object_detection_confidence` | float | 0.42 | Min camera detection confidence threshold |

### Association - Strict Matching (Primary Path)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.association_strict_matching` | bool | true | Enable strict multi-gate matching |
| `projection_utils_params.min_iou_threshold` | double | 0.11 | Base IoU threshold |
| `projection_utils_params.association_centroid_inner_box_fraction` | double | 0.85 | Fraction of detection box for centroid check |
| `projection_utils_params.association_min_inside_point_fraction` | double | 0.45 | Min fraction of points inside detection |
| `projection_utils_params.association_min_ar_consistency_score` | double | 0.30 | Min aspect ratio consistency score |
| `projection_utils_params.association_use_point_projection_rect_for_iou` | bool | true | Use projected point hull vs. AABB for IoU |

### Association - Distance-Adaptive Relaxation

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.far_iou_threshold_scale` | double | 0.5 | IoU multiplier for clusters > 30m |
| `projection_utils_params.medium_iou_threshold_scale` | double | 0.75 | IoU multiplier for clusters > 20m |
| `projection_utils_params.far_inside_fraction_scale` | double | 0.6 | Inside fraction scale for far clusters |
| `projection_utils_params.medium_inside_fraction_scale` | double | 0.8 | Inside fraction scale for medium-far clusters |

### Association - Scoring Weights

Weights must sum to 1.0 before depth adjustment.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.association_weight_iou` | double | 0.30 | Weight for IoU score |
| `projection_utils_params.association_weight_inside_fraction` | double | 0.28 | Weight for inside point fraction |
| `projection_utils_params.association_weight_ar` | double | 0.18 | Weight for aspect ratio |
| `projection_utils_params.association_weight_centroid` | double | 0.14 | Weight for centroid proximity |
| `projection_utils_params.association_weight_points` | double | 0.10 | Weight for point count |

### Assignment Solver

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.use_hungarian_assignment` | bool | true | Use Hungarian/Munkres for optimal assignment; false = greedy |

### Depth Enrichment (Optional)

From optional attribute assigner via `detections_3d_enriched` topic.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.depth_score_weight` | double | 0.15 | Weight of depth-proximity term |
| `projection_utils_params.depth_score_scale` | double | 5.0 | Exponential decay scale (m) for depth score |
| `projection_utils_params.min_depth_for_enrichment` | double | 0.1 | Min enriched depth (m) to accept |

### Class-Aware Size Prior (Pre-Match Penalty)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.enable_size_prior` | bool | true | Master toggle for class-specific size prior |
| `projection_utils_params.size_prior_weight` | double | 0.10 | Weight of size-prior term |
| `projection_utils_params.size_prior_scale` | double | 0.5 | Exponential decay scale (m); lower = stricter |
| `projection_utils_params.size_prior_classes` | array | [person, car, truck, bus, bicycle, motorcycle] | Class names |
| `projection_utils_params.size_prior_min_widths` | array | [0.2, 1.4, 2.0, 2.2, 0.3, 0.4] | Min width (m) per class |
| `projection_utils_params.size_prior_max_widths` | array | [0.8, 2.2, 3.0, 3.0, 0.8, 1.0] | Max width (m) per class |
| `projection_utils_params.size_prior_min_lengths` | array | [0.2, 3.0, 5.0, 8.0, 1.2, 1.5] | Min length (m) per class |
| `projection_utils_params.size_prior_max_lengths` | array | [0.8, 5.5, 14.0, 15.0, 2.0, 2.5] | Max length (m) per class |
| `projection_utils_params.size_prior_min_heights` | array | [0.8, 1.2, 2.0, 2.5, 0.8, 0.8] | Min height (m) per class |
| `projection_utils_params.size_prior_max_heights` | array | [2.2, 2.0, 4.5, 4.0, 1.8, 1.8] | Max height (m) per class |

### 3D Output Score Blend

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.detection_score_weight` | double | 0.6 | Weight of 2D detection score |
| `projection_utils_params.iou_score_weight` | double | 0.4 | Weight of IoU score |

### Quality Filtering (Post-Match, Per-Class Caps)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.quality_person_max_height_m` | float | 2.5 | Max height for person class (m) |
| `projection_utils_params.quality_person_max_footprint_xy_m` | float | 1.45 | Max xy footprint for person (m) |
| `projection_utils_params.quality_person_max_volume_m3` | float | 4.0 | Max volume for person (m^3) |
| `projection_utils_params.quality_vehicle_max_height_m` | float | 4.8 | Max height for vehicle class (m) |

### 3D Box Fitting

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.use_l_shaped_fitting` | bool | true | L-shaped for car/truck/bus; false = search-based for all |
| `projection_utils_params.orientation_search_step_degrees` | double | 2.0 | Fine search step size (degrees) |
| `projection_utils_params.orientation_coarse_step_multiplier` | double | 5.0 | Coarse step = fine_step *this |
| `projection_utils_params.orientation_fine_range_multiplier` | double | 2.5 | Fine search range = fine_step* this around best coarse angle |
| `projection_utils_params.l_shape_energy_variance_weight` | double | 0.5 | L-shape: combined = mean_edge_dist + this * variance |
| `projection_utils_params.ar_front_view_threshold` | double | 1.2 | Aspect ratio threshold for front-view disambiguation |
| `projection_utils_params.default_sample_point_count` | int | 64 | Sample point count for fitting |
| `projection_utils_params.min_points_for_fit` | int | 3 | Minimum points required for box fitting |
| `projection_utils_params.outlier_rejection_point_count` | int | 30 | Threshold for activating outlier rejection |
| `projection_utils_params.outlier_sigma_multiplier` | double | 4.5 | Clipping threshold (sigma) for outliers |
| `projection_utils_params.xy_extent_percentile_low` | double | 5.0 | Low percentile for xy bounds |
| `projection_utils_params.xy_extent_percentile_high` | double | 95.0 | High percentile for xy bounds |
| `projection_utils_params.z_extent_percentile_low` | double | 2.0 | Low percentile for z bounds |
| `projection_utils_params.z_extent_percentile_high` | double | 98.0 | High percentile for z bounds |

### Scoring Internals (Rarely Need Tuning)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.centroid_score_detection_scale` | double | 0.35 | Centroid score scale = this * hypot(box_w, box_h) |
| `projection_utils_params.point_score_saturation_count` | double | 120.0 | Point score saturation count (log-scale) |
| `projection_utils_params.association_centroid_distance_multiplier` | double | 1.5 | Early rejection distance multiplier |
| `projection_utils_params.single_point_bbox_margin_px` | double | 3.0 | Synthetic bbox half-size for single point (pixels) |
| `projection_utils_params.min_camera_z_distance` | double | 0.8 | Min camera Z distance (m) |
| `projection_utils_params.image_width` | int | 1280 | Image width for projection clipping |
| `projection_utils_params.image_height` | int | 1024 | Image height for projection clipping |

### Class-Aware Threshold Scaling

Applied on top of distance-adaptive thresholds.

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.person_inside_fraction_scale` | double | 0.85 | Relaxed for narrow persons |
| `projection_utils_params.vehicle_inside_fraction_scale` | double | 0.85 | Relaxed for wide vehicles |
| `projection_utils_params.person_iou_threshold_scale` | double | 0.90 | Relaxed IoU for persons |
| `projection_utils_params.vehicle_iou_threshold_scale` | double | 0.90 | Relaxed IoU for vehicles |
| `projection_utils_params.person_ar_consistency_scale` | double | 0.70 | Relaxed AR gate for persons |

### Support Checks

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.association_min_inside_points` | int | 2 | Absolute floor for inside-point count |
| `projection_utils_params.min_size_consistency_score` | double | 0.25 | Width/height ratio plausibility floor |
| `projection_utils_params.min_det_area_far_px2` | double | 400.0 | Min 2D detection area for far clusters (px^2); 0 = disabled |

### Marker Visualization

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `projection_utils_params.marker_lifetime_s` | double | 0.25 | Marker lifetime (s) |
| `projection_utils_params.marker_alpha` | double | 0.2 | Marker transparency |

### Quality Filter (Global, Pre-Class-Caps)

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `quality_filter_params.max_distance` | double (m) | 60.0 | Drop clusters beyond this range |
| `quality_filter_params.min_points` | int | 5 | Global minimum points |
| `quality_filter_params.min_height` | float (m) | 0.15 | Global minimum AABB height |
| `quality_filter_params.min_points_default` | int | 8 | Baseline min points |
| `quality_filter_params.min_points_far` | int | 8 | Min points when distance > 30m |
| `quality_filter_params.min_points_medium` | int | 10 | Min points when distance > 20m |
| `quality_filter_params.min_points_large` | int | 30 | Min points when volume > 8m^3 |
| `quality_filter_params.distance_threshold_far` | double (m) | 30.0 | Far distance threshold |
| `quality_filter_params.distance_threshold_medium` | double (m) | 20.0 | Medium distance threshold |
| `quality_filter_params.volume_threshold_large` | float (m^3) | 8.0 | Large volume threshold |
| `quality_filter_params.min_density` | float (pts/m^3) | 5.0 | Minimum points-per-unit-volume |
| `quality_filter_params.max_dimension` | float (m) | 15.0 | Max largest AABB edge |
| `quality_filter_params.max_aspect_ratio` | float | 15.0 | Max ratio of largest to smallest AABB edge |

### Cross-Camera Deduplication

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `cross_camera_dedup.enabled` | bool | true | Enable cross-camera duplicate removal |
| `cross_camera_dedup.min_lidar_index_overlap` | double | 0.5 | Intersection/min(\|A\|,\|B\|) overlap threshold |
| `cross_camera_dedup.weak_lidar_index_overlap` | double | 0.22 | Weak overlap with class match threshold |
| `cross_camera_dedup.max_center_dist_m` | double (m) | 1.8 | Max BEV center distance for merge |
| `cross_camera_dedup.min_bev_box_iou` | double | 0.16 | Min BEV axis-aligned IoU |
| `cross_camera_dedup.min_bev_box_iou_no_class` | double | 0.30 | Stricter BEV IoU when both lack class_id |

### Visualization Toggles

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `publish_bounding_box` | bool | true | Publish 3D bounding box markers |
| `publish_visualization` | bool | false | Publish debug point-cloud topics |
| `publish_image_visualization` | bool | false | Publish image visualization overlays |
| `bbox_markers_use_detection_stamp` | bool | false | Stamp bbox markers with batch detection time |
| `debug_logging` | bool | false | Enable verbose debug logging |

## Key Components

### 1. Class-Aware Filtering

**`filterCandidatesByClassAwareConstraints()`** runs after IoU matching and applies size/quality constraints (same defaults for all candidates). Constraint values are configured via `quality_filter_params` in `params.yaml`.

### 2. Multi-camera behavior

The node subscribes to **`deep_msgs::msg::MultiDetection2DArray`** (from deep_ros `deep_object_detection`), which contains one `Detection2DArray` per camera, each with its own `header.frame_id` and `detections[]`.

- **Per-camera processing**: For each camera in `camera_detections`, the node uses the **same** lidar cloud and a **copy** of the same cached cluster candidates. It runs the full pipeline (IoU assignment, class-aware filter, 3D box computation) independently for that camera.
- **Concatenation**: All 3D detections from all cameras are appended into a single `Detection3DArray` (`merged_detections3d`) and published on `/detection_3d`. Bounding box markers are namespaced by camera via `marker.ns = frame_id`.
- **Cross-camera deduplication**: When `cross_camera_dedup.enabled` is true (default), duplicate detections from multiple cameras are merged using LiDAR index overlap and BEV box IoU. This prevents the same physical object from producing multiple 3D detections.
- **Frame and sync requirements**: Each camera's `header.frame_id` must match an entry in `MultiCameraInfo`; detection-cloud stamp delta can be checked via `max_detection_cloud_stamp_delta` (cameras exceeding it are skipped).

## Key Functions

### Clustering Pipeline

The pipeline uses a single container type **`ClusterCandidate`** (indices + stats + optional match) so alignment cannot go stale:

1. **Cluster** - `SpatialAssociationCore::performClustering()`: Euclidean clustering, then merge.
2. **Build candidates** - `ProjectionUtils::buildCandidates(cloud, cluster_indices)`: one struct per cluster with indices and stats.
3. **IoU match** - `ProjectionUtils::assignCandidatesToDetectionsByIOU(cloud, candidates, ...)`: fills `candidate.match`, removes unmatched.
4. **Class-aware filter** - `ProjectionUtils::filterCandidatesByClassAwareConstraints(candidates, detections)`: reads quality thresholds from `ProjectionUtils::getParams()` (filled from `quality_filter_params` in `initializeParams()` / `setParams`).
5. **Box fitting** - boxes from `extractIndices(candidates)`, then `compute3DDetection(boxes, candidates, ...)` for class/score from `candidate.match`.

#### `SpatialAssociationCore::performClustering()`

1. **Euclidean Clustering**: Groups nearby points (standard or adaptive with multi-band support).
2. **Cluster Merging**: `mergeClusters()` - merges nearby clusters. Physics filtering is done in the node on candidates.

#### `ProjectionUtils::computeClusterStats()` / `buildCandidates()`

Computes statistics for each cluster:
- Centroid (3D position)
- Bounding box (min/max x, y, z)
- Point count

Used throughout the filtering pipeline.

### Detection Matching

#### `ProjectionUtils::assignCandidatesToDetectionsByIOU()`

Operates on `std::vector<ClusterCandidate>`: fills `candidate.match` for kept clusters, removes unmatched.

In **strict mode** (default), all gates must pass:
1. **Centroid**: Inside inner 85% of detection box
2. **IoU**: On projected point hull >= 0.11 (distance-scaled)
3. **Inside fraction**: >= 0.45 points inside detection (distance & class scaled)
4. **Aspect ratio**: Consistency >= 0.30
5. **Point support**: Tiered min (8-30 points depending on distance/volume)

Scoring: `0.30*IoU + 0.28*inside_frac + 0.18*AR + 0.14*centroid + 0.10*points`

Assignment uses **Hungarian algorithm** (optimal) by default, or greedy fallback when `use_hungarian_assignment: false`.

### Bounding Box Computation

#### `ProjectionUtils::computeClusterBoxes()`

Implementation is in `utils/cluster_box_utils.cpp` (`cluster_box` namespace):

- **L-shaped fitting** for car/truck/bus (default): edge-energy minimization for vehicle orientation recovery
- **Search-based fitting** for others: coarse (10 degree steps) then fine (2 degree steps) around best coarse angle
- **Percentile-based bounds**: xy [5%-95%], z [2%-98%]
- **Outlier rejection**: Sigma clipping (4.5 sigma) for clusters > 30 points

## Topic Configuration

### Subscribers
- `multi_camera_info` (remapped to e.g. `/multi_camera_sync/multi_camera_info`): **Single batched** camera calibration from the deep_ros camera_sync node. The node does not subscribe to individual per-camera `camera_info` topics.
- `multi_image` (remapped to `/multi_camera_sync/multi_image_compressed`): Batched **compressed** images from camera_sync, for image-based visualizations. Subscribed only when `publish_image_visualization: true`.
- `non_ground_cloud`: Non-ground filtered point cloud (from patchwork++)
- `detections` (`deep_msgs::msg::MultiDetection2DArray`): Per-camera 2D detections from deep_object_detection; see **Multi-camera behavior** above.
- `detections_3d_enriched` (`vision_msgs::msg::Detection3DArray`): Optional; for depth enrichment from attribute assigner.

### Camera name / frame_id (deep_ros compatibility)
- The same **camera name** (frame_id) must be used in three places so the node can associate detections with camera info and TF:
  1. **MultiCameraInfo**: each `camera_infos[i].header.frame_id` (set by camera_sync from its `camera_names` or from the source CameraInfo).
  2. **Detection2DArray**: `header.frame_id` (set by deep_object_detection per batched image; must match the camera name for that image).
  3. **TF**: the camera optical frame name in the TF tree (published by sensor_interfacing) must match so `lookupTransform(camera_frame_id, lidar_frame)` succeeds.
- Entries in MultiCameraInfo with empty `header.frame_id` are skipped and a throttled warning is logged.

### Transforms (TF)
- Camera-lidar transforms are **not** hardcoded. The node uses the TF tree (subscribes via `tf2_ros::Buffer`/`TransformListener`) to look up the transform from the lidar frame to each camera frame. These transforms are expected to be published by **sensor_interfacing** (e.g. static transforms or from the robot description).
- Parameter `lidar_frame` (default: `lidar_cc`) specifies the frame of the lidar point cloud. Use `lidar_cc` when the non-ground cloud comes from the center lidar (e.g. `/lidar_cc/velodyne_points` -> patchwork++ -> `non_ground_cloud`).

### Publishers
- `/detection_3d`: 3D detections (`vision_msgs::msg::Detection3DArray`)
- `/bounding_box`: 3D bounding box markers (if `publish_bounding_box: true`)
- `/filtered_lidar`: Colored cluster cloud (if `publish_visualization: true`)
- `/cluster_centroid`: Cluster centroids (if `publish_visualization: true`)

## Code Structure

```
spatial_association/
├── src/
│   ├── main.cpp                                # Standalone executable entry point
│   ├── spatial_association.cpp                  # ROS2 node implementation
│   └── spatial_association_core.cpp             # Core clustering logic
├── include/
│   └── spatial_association/
│       ├── spatial_association.hpp              # ROS2 node header
│       └── spatial_association_core.hpp         # Core class header
├── utils/
│   ├── src/
│   │   ├── projection_utils.cpp                # Projection, matching, filtering
│   │   ├── cluster_box_utils.cpp               # Box fitting and cluster stats
│   │   └── hungarian.cpp                       # Hungarian algorithm for assignment
│   └── include/
│       └── utils/
│           ├── projection_utils.hpp            # Projection utils header
│           ├── cluster_box_utils.hpp           # Box fitting header
│           └── hungarian.hpp                   # Hungarian algorithm header
├── config/
│   └── params.yaml                             # ROS2 parameters
├── launch/
│   └── spatial_association_launch.yaml         # Launch file (3 modes)
└── docs/
    └── DEVELOPING.md                           # This file
```

## Key Algorithms

### 1. Multi-Band Adaptive Euclidean Clustering

Generalizes two-band (close/far) to N distance bands, each with independent tolerance and min cluster size. Configured via `clustering_bands` (overrides legacy `close_threshold`/`close_tolerance_mult` when non-empty).

Default bands:
- 0-8m: tolerance * 1.3, min 15 points
- 8-20m: tolerance * 1.0, min 12 points
- 20-40m: tolerance * 1.5, min 10 points
- 40m+: tolerance * 2.0, min 8 points

Prevents fragmentation of close objects while maintaining precision for distant objects.

### 2. L-Shaped Fitting (Vehicles)

Alternative to search-based orientation fitting; fits L-shaped corner points (front bumper + side profile) for better vehicle orientation recovery. Used by default for car/truck/bus classes.

### 3. Search-Based Orientation Fitting

For non-vehicle classes:
1. **Coarse search**: Tests angles in 10 degree steps (0 to 90 degrees)
2. **Fine search**: Refines best angle in +/-5 degree range with 2 degree steps
3. **Edge energy minimization**: Finds orientation that minimizes distance to bounding box edges

### 4. Outlier Rejection

For clusters with > 30 points:
- Computes mean and standard deviation of rotated coordinates
- Clips points beyond 4.5 sigma to prevent outliers from skewing bounding box

### 5. Aspect Ratio Disambiguation

When aspect ratio < 1.2 (nearly square):
- Uses line-of-sight yaw (from centroid to sensor)
- Otherwise uses fitted orientation, choosing the direction closest to line-of-sight

### 6. Cross-Camera Deduplication

Merges duplicate detections from multiple camera views:
- LiDAR index overlap >= 0.5 (intersection/min set size)
- Weak overlap (0.22) + class match + close center (< 1.8m)
- BEV box IoU fallback (0.16 threshold, or 0.30 if no class)

## Threading Model

- **Main thread**: ROS2 executor (processes callbacks)
- **Worker thread**: Processes incoming point clouds (downsampling, clustering, candidate building)
- **Association thread**: Processes detections, runs IoU matching, publishes results
- **Synchronization**: Mutexes protect cloud cache, TF cache, detection queue

## Lifecycle

The node is a **ROS2 LifecycleNode** with three launch modes:

1. **Standalone (default)**: Node executable self-configures and self-activates
2. **Own Container**: Creates MT component container, loads composable node, starts lifecycle manager
3. **Load into Existing Container**: Integrates with perception_bringup container

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

**Solution**: Adjust clustering bands or increase base tolerance:

```yaml
euclid_params:
  cluster_tolerance: 0.50  # Increase from 0.40
  use_adaptive_clustering: true
  clustering_bands:
    tolerance_mults: [1.5, 1.2, 1.8, 2.5]  # Increase multipliers
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

3. Verify cluster counts match expectations
4. Check for false positives/negatives

## References

- **PCL Documentation**: http://pointclouds.org/documentation/
- **2D-3D IOU Paper**: https://arxiv.org/pdf/1908.03851

---
