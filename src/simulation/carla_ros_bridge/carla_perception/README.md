# carla_perception

Sensor data publishers for CARLA simulation.

These nodes spawn virtual sensors attached to the ego vehicle in CARLA and publish their data as ROS messages. Sensor positions are read from TF (published by `robot_state_publisher` from URDF) at activation time, so the URDF frame names must match the configured `frame_id` parameters.

## Nodes

### camera_publisher

Spawns RGB cameras in CARLA and publishes images. Supports multiple cameras configured via the `camera_names` parameter. Each camera's resolution, FOV, and intrinsics are configured via namespaced parameters (e.g., `front_camera.image_width`).

Camera intrinsics (K, P matrices) are published alongside images for use with image processing pipelines.

```bash
ros2 run carla_perception camera_publisher
```

**Publications:** `<camera_name>/image_raw` (`sensor_msgs/Image`), `<camera_name>/camera_info` (`sensor_msgs/CameraInfo`)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `carla_host` | string | `localhost` | CARLA server hostname |
| `carla_port` | int | `2000` | CARLA server port |
| `carla_timeout` | double | `10.0` | Connection timeout in seconds |
| `role_name` | string | `ego_vehicle` | Role name of the ego vehicle to attach sensors to |
| `camera_names` | string[] | `['camera']` | List of camera sensor names to spawn |

Per-camera parameters (replace `<name>` with camera name):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `<name>.frame_id` | string | *required* | TF frame for camera position |
| `<name>.image_width` | int | *required* | Image width in pixels |
| `<name>.image_height` | int | *required* | Image height in pixels |
| `<name>.fov` | double | `90.0` | Horizontal field of view in degrees |
| `<name>.optical_frame` | bool | `true` | Whether frame_id uses optical convention (Z-forward) |
| `<name>.camera_matrix.data` | double[] | | 3x3 camera intrinsic matrix (row-major) |
| `<name>.distortion_coefficients.data` | double[] | | Distortion coefficients |
| `<name>.projection_matrix.data` | double[] | | 3x4 projection matrix (row-major) |

### lidar_publisher

Spawns LiDAR sensors in CARLA and publishes point clouds. Supports multiple LiDARs configured via the `lidar_names` parameter. Each LiDAR's range, channels, rotation frequency, and FOV are configured via namespaced parameters.

Point clouds are accumulated over a full rotation before publishing, ensuring each message contains a complete 360° scan (or the configured horizontal FOV).

```bash
ros2 run carla_perception lidar_publisher
```

**Publications:** `<lidar_name>/points` (`sensor_msgs/PointCloud2`)

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `carla_host` | string | `localhost` | CARLA server hostname |
| `carla_port` | int | `2000` | CARLA server port |
| `carla_timeout` | double | `10.0` | Connection timeout in seconds |
| `role_name` | string | `ego_vehicle` | Role name of the ego vehicle to attach sensors to |
| `lidar_names` | string[] | `['lidar']` | List of LiDAR sensor names to spawn |

Per-LiDAR parameters (replace `<name>` with LiDAR name):

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `<name>.frame_id` | string | *required* | TF frame for LiDAR position |
| `<name>.range` | double | *required* | Maximum detection range in meters |
| `<name>.rotation_frequency` | double | *required* | Scan rate in Hz |
| `<name>.points_per_channel` | int | *required* | Points per channel per rotation |
| `<name>.channels` | int | `32` | Number of laser channels |
| `<name>.horizontal_fov` | double | `360.0` | Horizontal field of view in degrees |
| `<name>.upper_fov` | double | `10.0` | Upper vertical FOV limit in degrees |
| `<name>.lower_fov` | double | `-30.0` | Lower vertical FOV limit in degrees |
| `<name>.noise_stddev` | double | `0.0` | Distance noise standard deviation |
| `<name>.dropoff_general_rate` | double | `0.0` | Random point dropout rate (0.0-1.0) |

### bbox_publisher

Publishes ground truth 3D bounding boxes for all vehicles and pedestrians in the CARLA world. Useful for perception algorithm development and validation.

```bash
ros2 run carla_perception bbox_publisher
```

**Publications:** `detections_3d` (`vision_msgs/Detection3DArray`), `tracked_detections_3d` (`vision_msgs/Detection3DArray`)

**ObjectHypothesisWithPose format** — each `Detection3D.results` list is populated as follows:

| Object type | `results[0]` | `results[1+]` |
|---|---|---|
| Vehicle | `class_id="vehicle"`, `score=1.0` | One entry per active signal: `"left_blinker"`, `"right_blinker"`, `"brake"`, `"reverse"` |
| Pedestrian | `class_id="pedestrian"`, `score=1.0` | — |
| Traffic light | `class_id="traffic_light"`, `score=1.0` | `class_id="red"/"yellow"/"green"/"unknown"`, `score=1.0` |

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `carla_host` | string | `localhost` | CARLA server hostname |
| `carla_port` | int | `2000` | CARLA server port |
| `carla_timeout` | double | `10.0` | Connection timeout in seconds |
| `publish_rate` | double | `10.0` | Publish rate in Hz |
| `frame_id` | string | `base_link` | Frame ID for detections |
| `include_vehicles` | bool | `true` | Include vehicle detections |
| `include_pedestrians` | bool | `true` | Include pedestrian detections |
