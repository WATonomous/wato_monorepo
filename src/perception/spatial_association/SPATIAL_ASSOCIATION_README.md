# How to run the Spatial Association ROS Node

1. build the perception container

`watod build perception`

2. attach a shell or open the container and run (in mode 'develop')

`colcon build spatial_association`

3. launch the node

`ros2 launch spatial_association spatial_association_launch.yaml`

## Key Parameters (config/params.yaml)

### Euclidean Clustering Parameters

- `euclid_params.cluster_tolerance`: Maximum distance between points in the same cluster (meters); increase to merge nearby clusters, decrease for tighter clustering.
- `euclid_params.min_cluster_size`: Minimum points required per cluster; increase to filter out small noise clusters, decrease to detect smaller objects.
- `euclid_params.max_cluster_size`: Maximum points allowed per cluster; increase to handle larger objects, decrease to filter out ground artifacts.
- `euclid_params.use_adaptive_clustering`: Enable distance-adaptive clustering; true uses larger tolerance for close objects to prevent fragmentation, false uses uniform tolerance.
- `euclid_params.close_threshold`: Distance threshold (meters) for classifying points as "close"; objects within this distance use multiplied tolerance.
- `euclid_params.close_tolerance_mult`: Multiplier for close object tolerance (e.g., 1.5x = 0.30m for close objects when base is 0.20m); increase to merge more fragmented close objects.

### Density Filtering Parameters (Legacy - Not Used with Improved Filter)

- `density_filter_params.density_weight`: Weight for density score in legacy filter (0-1); currently unused with improved physics-based filtering.
- `density_filter_params.size_weight`: Weight for size score in legacy filter (0-1); currently unused with improved physics-based filtering.
- `density_filter_params.distance_weight`: Weight for distance score in legacy filter (0-1); currently unused with improved physics-based filtering.
- `density_filter_params.score_threshold`: Minimum score threshold for legacy filter (0-1); currently unused with improved physics-based filtering.

### Other Parameters

- `merge_threshold`: Maximum distance between cluster centroids to merge (meters); increase to merge more clusters, decrease to keep clusters separate.
- `object_detection_confidence`: Minimum confidence threshold for camera detections (0-1); increase to use only high-confidence detections, decrease to include lower-confidence matches.
- `voxel_size`: Voxel grid leaf size for downsampling (meters); increase for faster processing with lower resolution, decrease for higher resolution at more compute cost.
- `publish_visualization`: Enable visualization topics (bounding boxes, colored clusters, centroids); true for debugging/visualization, false for production.
- `debug_logging`: Enable verbose diagnostic logging; true for detailed cluster statistics, false for quiet operation.

## Topic Configuration

### Subscribers

- `non_ground_cloud_topic`: Non-ground filtered point cloud from patchwork (`/non_ground_cloud` by default).
- `detections_topic`: Batch detection messages from cameras (`/batched_camera_message` by default).
- `camera_info_topic_front_`, `camera_info_topic_left_`, `camera_info_topic_right_`: Camera calibration info topics.

### Publishers

- `detection_3d_topic`: 3D detections output (`/detection_3d` by default).
- `bounding_box_topic`: Visualization markers (`/bounding_box` by default, requires `publish_visualization: true`).
- `filtered_lidar_topic`: Colored cluster point cloud (`/filtered_lidar` by default, requires `publish_visualization: true`).
- `cluster_centroid_topic`: Cluster centroids (`/cluster_centroid` by default, requires `publish_visualization: true`).

## Improved Cluster Filtering System

The node uses a **physics-based cluster filtering system** that replaces the old weighted scoring approach. This system automatically filters clusters using real-world object constraints:

- **Noise filtering**: Removes clusters with < 5 points or < 15cm in any dimension.
- **Geometry filtering**: Validates object sizes, aspect ratios, and point density against physical constraints.
- **Quality filtering**: Uses distance-adaptive thresholds (stricter at close range, more lenient at distance).

The filtering thresholds are based on real-world measurements:
- Cars: 1.0-3.5m height, 2.5-8.0m length, 1.4-2.8m width
- Pedestrians: 0.8-2.2m height, < 1.0m width
- Maximum detection range: 60m (configurable)

For detailed information about the filtering system, see [DEVELOPING.md](./DEVELOPING.md).

