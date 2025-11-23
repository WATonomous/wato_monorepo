# Spatial Association Node - Developer Guide

## Overview

The `spatial_association` node is a ROS2 perception node that performs 3D object detection by combining LiDAR point cloud data with 2D camera detections. It clusters LiDAR points, filters them using physics-based constraints, and associates them with camera detections using IoU (Intersection over Union) matching.

## Architecture

The node is structured in three main layers:

### 1. ROS2 Node Layer (`spatial_association`)
- **File**: `src/spatial_association.cpp`, `include/spatial_association.hpp`
- Handles ROS2 communication (subscribers, publishers, parameters)
- Coordinates between LiDAR processing and camera detection matching
- Manages TF transforms between sensor frames

### 2. Core Processing Layer (`SpatialAssociationCore`)
- **File**: `src/spatial_association_core.cpp`, `include/spatial_association_core.hpp`
- Performs point cloud downsampling (voxel grid)
- Executes clustering pipeline (Euclidean clustering → filtering → merging)
- Stateless, reusable library (no ROS dependencies)

### 3. Utility Layer (`ProjectionUtils`)
- **File**: `utils/src/projection_utils.cpp`, `utils/include/utils/projection_utils.hpp`
- Point cloud clustering algorithms
- **Improved cluster filtering system** (physics-based constraints)
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
│ Ground Noise    │
│ Filtering       │
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Quality Filter  │ 
└────────┬────────┘
         │
         ▼
┌─────────────────┐
│ Cluster Merging │
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
                        │ 3D Bounding     │
                        │ Boxes &         │
                        │ Detections      │
                        └─────────────────┘
```


### Key Components

#### 1. `ObjectConstraints` Struct

Defines physical constraints based on real-world measurements:

```cpp
struct ObjectConstraints {
  // Cars
  constexpr static float kCarMinHeight = 1.0f;
  constexpr static float kCarMaxHeight = 3.5f;
  constexpr static float kCarMinLength = 2.5f;
  constexpr static float kCarMaxLength = 8.0f;
  constexpr static float kCarMinWidth = 1.4f;
  constexpr static float kCarMaxWidth = 2.8f;
  
  // Pedestrians/Cyclists
  constexpr static float kPedestrianMinHeight = 0.8f;
  constexpr static float kPedestrianMaxHeight = 2.2f;
  constexpr static float kPedestrianMaxWidth = 1.0f;
  
  // General constraints
  constexpr static float kMinPointDensity = 5.0f;     // points per m³
  constexpr static float kMaxPointDensity = 1000.0f;  // points per m³
  constexpr static int kMinPointsSmallObject = 10;
  constexpr static int kMinPointsLargeObject = 30;
  constexpr static float kMaxAspectRatioXY = 8.0f;    // Length/Width
  constexpr static float kMaxAspectRatioZ = 15.0f;    // Horizontal/Height (for poles)
};
```

#### 2. `ClusterFilter` Class

Multi-stage filtering with three stages:

**Stage 1: Noise Filtering** (`filterNoise`)
- Removes clusters with < 5 points
- Filters extremely small objects (< 15cm in any dimension)

**Stage 2: Geometry-Based Filtering** (`filterByGeometry`)
- Checks for unrealistic sizes (> 15m length, > 4m width, > 5m height)
- Validates aspect ratios (prevents wall/fence artifacts)
- Filters thin vertical poles (false positives)
- Validates point density (too sparse or too dense = invalid)

**Stage 3: Quality Filtering** (`filterByQuality`)
- Distance-adaptive point thresholds:
  - Distance > 30m: min 8 points
  - Distance > 20m: min 12 points
  - Close range: min 10 points (small) or 30 points (large objects)
- Completeness check (expected vs actual points)
- Maximum distance cutoff (default: 60m)

#### 3. Filtering Functions

**`filterClustersByPhysicsConstraints()`** (Currently Used)
- Physics-based filtering with distance-adaptive thresholds
- Single-pass filtering with improved logic
- Validates geometry, density, aspect ratios, and distance constraints
- Simpler API, good performance

**`filterClusterByQuality()`** (Alternative)
- Multi-stage filtering with detailed statistics
- Optional debug output showing filtering breakdown
- More granular control over each stage
- Use when you need detailed filtering metrics

### Usage

The physics-based filter is automatically used in `SpatialAssociationCore::performClustering()`:

```cpp
// In spatial_association_core.cpp
ProjectionUtils::filterClustersByPhysicsConstraints(
    cluster_stats,
    cluster_indices,
    60.0,   // max_distance in meters
    // ... additional physics-based parameters
);
```

To use the multi-stage version with debug output:

```cpp
ProjectionUtils::filterClusterByQuality(
    cluster_stats,
    cluster_indices,
    60.0,   // max_distance
    true);  // enable_debug
```

## Key Functions

### Clustering Pipeline

#### `SpatialAssociationCore::performClustering()`

Main clustering function that executes the full pipeline:

1. **Euclidean Clustering**: Groups nearby points
   - Standard: `euclideanClusterExtraction()`
   - Adaptive: `adaptiveEuclideanClusterExtraction()` (uses larger tolerance for close objects)

2. **Ground Noise Filtering**: `filterGroundNoise()`
   - Removes clusters that are too low or too flat
   - Filters ground plane artifacts

3. **Quality Filtering**: `filterClustersByPhysicsConstraints()`
   - Physics-based filtering with distance-adaptive thresholds

4. **Cluster Merging**: `mergeClusters()`
   - Merges nearby clusters that likely belong to the same object
   - Validates aspect ratios before merging

#### `ProjectionUtils::computeClusterStats()`

Computes statistics for each cluster:
- Centroid (3D position)
- Bounding box (min/max x, y, z)
- Point count

Used throughout the filtering pipeline.

### Detection Matching

#### `ProjectionUtils::computeHighestIOUCluster()`

Matches 3D LiDAR clusters with 2D camera detections:
1. Projects 3D cluster bounding box to 2D image plane
2. Computes IoU with each camera detection
3. Keeps only clusters with IoU ≥ `kMinIOUThreshold` (0.15)

#### `ProjectionUtils::computeMaxIOU8Corners()`

Projects 8 corners of 3D bounding box to 2D and computes maximum IoU.

### Bounding Box Computation

#### `ProjectionUtils::computeClusterBoxes()`

Computes oriented 3D bounding boxes:
- Uses search-based fitting for orientation
- Handles tall objects (cuts bottom portion for orientation)
- Returns `Box3D` with center, size, and yaw

## Configuration Parameters

### ROS2 Parameters (`config/params.yaml`)

#### Euclidean Clustering
```yaml
euclid_params:
  cluster_tolerance: 0.20        # Max distance between points in same cluster (m)
  min_cluster_size: 20          # Minimum points per cluster
  max_cluster_size: 800         # Maximum points per cluster
  use_adaptive_clustering: true # Use distance-adaptive tolerance
  close_threshold: 10.0         # Distance threshold for "close" objects (m)
  close_tolerance_mult: 1.5     # Multiplier for close objects (1.5x = 0.30m)
```

#### Physics-Based Filtering Parameters
The `filterClustersByPhysicsConstraints()` function uses physics-based constraints defined in the code. These are configured through ROS2 parameters in `params.yaml` (see Other Parameters section).

#### Other Parameters
```yaml
merge_threshold: 0.30                    # Max distance between centroids to merge (m)
object_detection_confidence: 0.40        # Min camera detection confidence
voxel_size: 0.1                         # Voxel grid leaf size (m)
publish_visualization: false            # Enable visualization topics
debug_logging: false                     # Enable verbose logging
```

### Topic Configuration

#### Subscribers
- `/non_ground_cloud`: Non-ground filtered point cloud (from patchwork)
- `/batched_camera_message`: Batch detection messages from cameras
- `/CAM_FRONT/camera_info`, `/CAM_FRONT_LEFT/camera_info`, `/CAM_FRONT_RIGHT/camera_info`: Camera calibration

#### Publishers
- `/detection_3d`: 3D detections (`vision_msgs::msg::Detection3DArray`)
- `/bounding_box`: Visualization markers (if `publish_visualization: true`)
- `/filtered_lidar`: Colored cluster cloud (if `publish_visualization: true`)
- `/cluster_centroid`: Cluster centroids (if `publish_visualization: true`)

## Code Structure

```
spatial_association/
├── src/
│   ├── spatial_association.cpp          # ROS2 node implementation
│   └── spatial_association_core.cpp     # Core clustering logic
├── include/
│   ├── spatial_association.hpp          # ROS2 node header
│   └── spatial_association_core.hpp     # Core class header
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

Splits point cloud into "close" and "far" regions:
- **Close** (< `close_threshold`): Uses `base_tolerance * close_tolerance_mult`
- **Far** (≥ `close_threshold`): Uses `base_tolerance`

Prevents fragmentation of close objects while maintaining precision for distant objects.

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

### Enable Filtering Statistics

To see detailed filtering breakdown, use `filterClusterByQuality()` with debug enabled:

```cpp
ProjectionUtils::filterClusterByQuality(
    cluster_stats,
    cluster_indices,
    60.0,
    true);  // enable_debug
```

Output example:
```
Cluster Filtering Results:
  Input: 45
  Removed (noise): 12
  Removed (geometry): 8
  Removed (density): 5
  Removed (distance): 3
  Output: 17
```

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
- Default: `0.1m` voxel size
- Reduces point cloud size significantly
- Trade-off: Lower resolution vs. faster processing

### Cluster Size Limits
- `min_cluster_size: 20`: Prevents tiny noise clusters
- `max_cluster_size: 800`: Prevents processing huge clusters (likely ground artifacts)

### Adaptive Clustering
- Reduces fragmentation of close objects
- Improves detection quality for nearby vehicles/pedestrians

## Common Issues & Solutions

### Issue: Too Many False Positives

**Solution**: Adjust filtering thresholds in `ObjectConstraints`:
- Increase `kMinPointDensity` (currently 5.0)
- Decrease `kMaxAspectRatioXY` (currently 8.0)
- Adjust distance-adaptive thresholds in `filterByQuality()`

### Issue: Missing Valid Objects

**Solution**: Relax filtering constraints:
- Decrease `kMinPointDensity`
- Increase `kMaxAspectRatioXY`
- Lower minimum point thresholds in distance-adaptive logic

### Issue: Clusters Too Fragmented

**Solution**: Increase cluster tolerance or use adaptive clustering:
```yaml
euclid_params:
  cluster_tolerance: 0.25  # Increase from 0.20
  use_adaptive_clustering: true
  close_tolerance_mult: 2.0  # Increase from 1.5
```

### Issue: TF Transform Errors

**Solution**: Ensure TF tree is properly configured:
- Check that `lidar_top_frame` parameter matches actual frame name
- Verify camera frames match camera info topics
- Use `ros2 run tf2_ros tf2_echo` to debug transforms

## Testing

### Unit Testing

Test individual components:
```cpp
// Test filtering
std::vector<ClusterStats> stats = ...;
std::vector<pcl::PointIndices> clusters = ...;
ProjectionUtils::filterClustersByPhysicsConstraints(stats, clusters, 60.0, /* ... other params ... */);
```

### Integration Testing

1. Launch node with test data
2. Check output topics:
   ```bash
   ros2 topic echo /detection_3d
   ```
3. Verify cluster counts match expectations
4. Check for false positives/negatives


### Code Improvements

1. **Stats Recomputation**: Currently stats are not recomputed after Stage 1 filtering in `filterClusterByQuality()`. Consider recomputing for more accurate Stage 2/3 filtering.

2. **Parameter Exposure**: Expose `ObjectConstraints` values as ROS2 parameters for easier tuning.

3. **Filter Selection**: Add parameter to choose between `filterClustersByPhysicsConstraints()` and `filterClusterByQuality()`.

## References

- **PCL Documentation**: http://pointclouds.org/documentation/
- **2D-3D IOU Paper**: https://arxiv.org/pdf/1908.03851

---

