#include "spatial_association.hpp"
#include "gpu_pipeline.hpp"
#include "cuda_utils.hpp"

spatial_association::spatial_association() : Node("spatial_association") {
  initializeParams();

  // Initialize working PCL objects for performance optimization
  working_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  working_downsampled_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  working_colored_cluster_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  working_centroid_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  
  // REMOVED: CPU voxel filter - GPU pipeline handles downsampling
  // OLD CODE (removed):
  //   voxel_filter_.setLeafSize(0.2f, 0.2f, 0.2f);
  
  // Initialize GPU pipeline with pre-allocated buffers (optional)
  if (use_gpu_pipeline_) {
    int max_points = 200000;  // Adjust based on typical point cloud size
    int max_clusters = 4000;
    if (!initializeGpuPipeline(max_points, max_clusters)) {
      RCLCPP_WARN(this->get_logger(), "GPU pipeline initialization failed, will use CPU fallback");
      use_gpu_pipeline_ = false;
    } else {
      cudaDeviceProp prop{};
      if (cudaGetDeviceProperties(&prop, 0) == cudaSuccess) {
        RCLCPP_INFO(this->get_logger(),
                    "GPU pipeline initialized on device '%s' (CC %d.%d), max_points=%d, max_clusters=%d",
                    prop.name, prop.major, prop.minor, max_points, max_clusters);
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "GPU pipeline initialized (device query failed), max_points=%d, max_clusters=%d",
                    max_points, max_clusters);
      }
    }
  } else {
    RCLCPP_WARN(this->get_logger(), "GPU pipeline disabled via parameter; using CPU clustering");
  }

  // SUBSCRIBERS
  // -------------------------------------------------------------------------------------------------

  non_ground_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      non_ground_cloud_topic_, 10, std::bind(&spatial_association::nonGroundCloudCallback, this, std::placeholders::_1));

  batch_dets_sub_ = this->create_subscription<camera_object_detection_msgs::msg::BatchDetection>(
      detections_topic_, 10, std::bind(&spatial_association::multiDetectionsCallback, this, std::placeholders::_1));

  auto info_qos = rclcpp::SensorDataQoS();

  camera_info_sub_front_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_front_, info_qos,
        std::bind(&spatial_association::multiCameraInfoCallback, this, std::placeholders::_1));

  camera_info_sub_left_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_left_, info_qos,
        std::bind(&spatial_association::multiCameraInfoCallback, this, std::placeholders::_1));
    
  camera_info_sub_right_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_right_, info_qos,
        std::bind(&spatial_association::multiCameraInfoCallback, this, std::placeholders::_1));

  // PUBLISHERS
  // --------------------------------------------------------------------------------------------------

  // Always create the main detection publisher
  detection_3d_pub_ = 
      this->create_publisher<vision_msgs::msg::Detection3DArray>(detection_3d_topic_, 10);
  bounding_box_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(bounding_box_topic_, 10);  

  // Only create visualization publishers if enabled
  if (publish_visualization_) {
    filtered_lidar_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_lidar_topic_, 10);
    cluster_centroid_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(cluster_centroid_topic_, 10);
    RCLCPP_INFO(this->get_logger(), "Visualization publishers enabled");
  } else {
    RCLCPP_INFO(this->get_logger(), "Visualization publishers disabled - only detection_3d will be published");
  }

  // Optional dual bbox publishers for debugging orientation methods
  if (bbox_debug_dual_publish_) {
    bbox_minarea_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(bbox_minarea_topic_, 10);
    bbox_pca2d_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>(bbox_pca2d_topic_, 10);
    RCLCPP_INFO(this->get_logger(), "Dual bbox debug publishers enabled: minarea='%s', pca2d='%s'",
                bbox_minarea_topic_.c_str(), bbox_pca2d_topic_.c_str());
  }

  // TIMER for LiDAR-only clustering (runs periodically when point clouds are available)
  // This allows publishing bounding boxes even without camera detections
  lidar_clustering_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(100),  // 10 Hz
      std::bind(&spatial_association::lidarClusteringTimerCallback, this));
  RCLCPP_INFO(this->get_logger(), "LiDAR clustering timer initialized (10 Hz)");
  

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

spatial_association::~spatial_association() {
  // Cleanup GPU pipeline (frees all device memory)
  cleanupGpuPipeline();
  if (debug_logging_) {
    RCLCPP_DEBUG(this->get_logger(), "GPU pipeline cleaned up");
  }
}

void spatial_association::initializeParams() {

  this->declare_parameter<std::string>("camera_info_topic_front_", "/CAM_FRONT/camera_info");
  this->declare_parameter<std::string>("camera_info_topic_left_", "/CAM_FRONT_LEFT/camera_info");
  this->declare_parameter<std::string>("camera_info_topic_right_", "/CAM_FRONT_RIGHT/camera_info");

  this->declare_parameter<std::string>("lidar_topic", "/LIDAR_TOP");
  this->declare_parameter<std::string>("non_ground_cloud_topic", "/non_ground_cloud");
  this->declare_parameter<std::string>("detections_topic", "/batched_camera_message");

  this->declare_parameter<std::string>("filtered_lidar_topic", "/filtered_lidar");
  this->declare_parameter<std::string>("cluster_centroid_topic", "/cluster_centroid");
  this->declare_parameter<std::string>("bounding_box_topic", "/bounding_box");
  this->declare_parameter<std::string>("detection_3d_topic", "/detection_3d");
  // Optional debug bbox topics
  this->declare_parameter<std::string>("bbox_minarea_topic", "/bounding_box_minarea");
  this->declare_parameter<std::string>("bbox_pca2d_topic", "/bounding_box_pca2d");

  this->declare_parameter<std::string>("lidar_top_frame", "LIDAR_TOP");

  this->declare_parameter<bool>("publish_visualization", true);
  this->declare_parameter<bool>("debug_logging", false);
  this->declare_parameter<bool>("use_gpu_pipeline", false);

  // Bounding box orientation control params
  this->declare_parameter<std::string>("bbox_orientation_method", "min_area");
  this->declare_parameter<double>("pca_reliability_min_ratio", 0.6);
  this->declare_parameter<int>("min_cluster_size_for_pca", 20);
  this->declare_parameter<bool>("bbox_debug_dual_publish", false);


  // Euclidean Clustering Parameters
  this->declare_parameter<double>("euclid_params.cluster_tolerance", 0.5);
  this->declare_parameter<int>("euclid_params.min_cluster_size", 50);
  this->declare_parameter<int>("euclid_params.max_cluster_size", 700);

  // Density Filtering Parameters
  this->declare_parameter<double>("density_filter_params.density_weight", 0.6);
  this->declare_parameter<double>("density_filter_params.size_weight", 0.8);
  this->declare_parameter<double>("density_filter_params.distance_weight", 0.7);
  this->declare_parameter<double>("density_filter_params.score_threshold", 0.6);

  this->declare_parameter<double>("merge_threshold", 0.3);

  this->declare_parameter<float>("object_detection_confidence", 0.4);

  // Get parameters
  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();

  camera_info_topic_front_ = this->get_parameter("camera_info_topic_front_").as_string();
  camera_info_topic_right_ = this->get_parameter("camera_info_topic_right_").as_string();
  camera_info_topic_left_ = this->get_parameter("camera_info_topic_left_").as_string();

  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  non_ground_cloud_topic_ = this->get_parameter("non_ground_cloud_topic").as_string();
  detections_topic_ = this->get_parameter("detections_topic").as_string();

  filtered_lidar_topic_ = this->get_parameter("filtered_lidar_topic").as_string();
  cluster_centroid_topic_ = this->get_parameter("cluster_centroid_topic").as_string();
  bounding_box_topic_ = this->get_parameter("bounding_box_topic").as_string();
  detection_3d_topic_ = this->get_parameter("detection_3d_topic").as_string();
  bbox_minarea_topic_ = this->get_parameter("bbox_minarea_topic").as_string();
  bbox_pca2d_topic_ = this->get_parameter("bbox_pca2d_topic").as_string();

  lidar_frame_ = this->get_parameter("lidar_top_frame").as_string();


  euclid_cluster_tolerance_ = this->get_parameter("euclid_params.cluster_tolerance").as_double();
  euclid_min_cluster_size_ = this->get_parameter("euclid_params.min_cluster_size").as_int();
  euclid_max_cluster_size_ = this->get_parameter("euclid_params.max_cluster_size").as_int();

  density_weight_ = this->get_parameter("density_filter_params.density_weight").as_double();
  size_weight_ = this->get_parameter("density_filter_params.size_weight").as_double();
  distance_weight_ = this->get_parameter("density_filter_params.distance_weight").as_double();
  score_threshold_ = this->get_parameter("density_filter_params.score_threshold").as_double();

  merge_threshold_ = this->get_parameter("merge_threshold").as_double();

  object_detection_confidence_ = this->get_parameter("object_detection_confidence").as_double();

  // Read bbox orientation params
  bbox_orientation_method_ = this->get_parameter("bbox_orientation_method").as_string();
  pca_reliability_min_ratio_ = this->get_parameter("pca_reliability_min_ratio").as_double();
  min_cluster_size_for_pca_ = this->get_parameter("min_cluster_size_for_pca").as_int();
  bbox_debug_dual_publish_ = this->get_parameter("bbox_debug_dual_publish").as_bool();
  debug_logging_ = this->get_parameter("debug_logging").as_bool();
  use_gpu_pipeline_ = this->get_parameter("use_gpu_pipeline").as_bool();

  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Debug logging is ENABLED for spatial_association");
  }
  RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}

void spatial_association::multiCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  const auto frame = msg->header.frame_id;
  camInfoMap_[frame] = msg;
}

void spatial_association::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // store the lidar msg
  latest_lidar_msg_ = *msg;
  filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL using working object
  pcl::fromROSMsg(latest_lidar_msg_, *working_cloud_);

  // Voxel downsampling is now handled by GPU pipeline in performClustering()
  // Store the cloud directly for GPU processing
  filtered_point_cloud_ = working_cloud_;
}

void spatial_association::nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received non-ground cloud with %d points", msg->width * msg->height);
  
  // Store the non-ground cloud message from patchwork
  latest_lidar_msg_ = *msg;
  filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL using working object
  pcl::fromROSMsg(latest_lidar_msg_, *working_cloud_);

  if (working_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty non-ground cloud");
    return;
  }

  // REMOVED: CPU voxel filtering - GPU pipeline will handle downsampling
  // OLD CODE (removed):
  //   voxel_filter_.setInputCloud(working_cloud_);
  //   voxel_filter_.filter(*working_downsampled_cloud_);
  //   filtered_point_cloud_ = working_downsampled_cloud_;

  // NEW CODE: Use raw cloud, GPU will downsample during clustering
  filtered_point_cloud_ = working_cloud_;
 
  RCLCPP_INFO(this->get_logger(), "Processed non-ground cloud: %zu points (ready for GPU)", 
              filtered_point_cloud_->size());

}

std::vector<ProjectionUtils::ClusterStats> spatial_association::performClustering(
    std::vector<pcl::PointIndices>& cluster_indices) {
  // Clear previous clusters
  cluster_indices.clear();

  if (!filtered_point_cloud_ || filtered_point_cloud_->empty()) {
    return {};
  }

  int N = static_cast<int>(filtered_point_cloud_->size());

  // ========== STEP 1: Convert PCL cloud to contiguous float array ==========
  std::vector<float> xyz;
  xyz.reserve(N * 3);
  for (size_t i = 0; i < filtered_point_cloud_->size(); ++i) {
    xyz.push_back(filtered_point_cloud_->points[i].x);
    xyz.push_back(filtered_point_cloud_->points[i].y);
    xyz.push_back(filtered_point_cloud_->points[i].z);
  }

  // ========== STEP 2: Setup GPU parameters ==========
  GPUParams params;
  params.voxel_leaf_size_x = 0.2f;
  params.voxel_leaf_size_y = 0.2f;
  params.voxel_leaf_size_z = 0.2f;
  params.cluster_tolerance = euclid_cluster_tolerance_;
  params.min_cluster_size = euclid_min_cluster_size_;
  params.max_cluster_size = euclid_max_cluster_size_;

  // ========== STEP 3: Run clustering (GPU preferred) ==========
  std::vector<int> labels;
  std::vector<GPUClusterStats> gpu_clusters;
  
  bool gpu_ok = false;
  if (use_gpu_pipeline_) {
    RCLCPP_INFO(get_logger(), "Calling GPU pipeline with %d points", N);
    gpu_ok = runGpuPipeline(xyz.data(), N, params, labels, gpu_clusters);
    if (!gpu_ok) {
      RCLCPP_ERROR(get_logger(), "GPU pipeline call returned false - falling back to CPU clustering");
    }
  }

  if (!gpu_ok) {
    // ========== FALLBACK: Use CPU implementation ==========
    ProjectionUtils::euclideanClusterExtraction(filtered_point_cloud_, 
                                                euclid_cluster_tolerance_,
                                                euclid_min_cluster_size_, 
                                                euclid_max_cluster_size_,
                                                cluster_indices);
    auto cluster_stats = ProjectionUtils::computeClusterStats(filtered_point_cloud_, cluster_indices);
    ProjectionUtils::filterClusterbyDensity(filtered_point_cloud_, cluster_indices, 
                                            density_weight_, size_weight_, 
                                            distance_weight_, score_threshold_);
    ProjectionUtils::mergeClusters(cluster_indices, filtered_point_cloud_, cluster_stats, merge_threshold_);
    cluster_stats = ProjectionUtils::computeClusterStats(filtered_point_cloud_, cluster_indices);
    return cluster_stats;
  }

  // ========== STEP 4: Rebuild pcl::PointIndices from labels ==========
  RCLCPP_INFO(get_logger(), "GPU pipeline returned %zu labels, %zu clusters", 
               labels.size(), gpu_clusters.size());
  
  int max_cluster_id = -1;
  for (int label : labels) {
    if (label > max_cluster_id) {
      max_cluster_id = label;
    }
  }

  if (max_cluster_id < 0) {
    RCLCPP_WARN(get_logger(), "No valid clusters found after GPU clustering (max_cluster_id=%d)", 
                max_cluster_id);
    return {};
  }
  
  RCLCPP_DEBUG(get_logger(), "Found max_cluster_id=%d, building cluster_indices", max_cluster_id);

  cluster_indices.resize(max_cluster_id + 1);
  for (auto& cluster : cluster_indices) {
    cluster.indices.clear();
  }

  // Populate cluster_indices from labels
  for (size_t i = 0; i < labels.size(); ++i) {
    int cluster_id = labels[i];
    if (cluster_id >= 0) {
      cluster_indices[cluster_id].indices.push_back(static_cast<int>(i));
    }
  }

  // Remove empty clusters
  cluster_indices.erase(
    std::remove_if(cluster_indices.begin(), cluster_indices.end(),
                   [](const pcl::PointIndices& ci) { return ci.indices.empty(); }),
    cluster_indices.end()
  );

  // ========== STEP 5: Convert GPUClusterStats to CPU ClusterStats ==========
  std::vector<ProjectionUtils::ClusterStats> cluster_stats;
  cluster_stats.reserve(gpu_clusters.size());
  
  for (const auto& gpu_stat : gpu_clusters) {
    ProjectionUtils::ClusterStats stat;
    stat.centroid = Eigen::Vector4f(gpu_stat.centroid_x, 
                                    gpu_stat.centroid_y, 
                                    gpu_stat.centroid_z, 
                                    1.0f);
    stat.min_x = gpu_stat.min_x;
    stat.max_x = gpu_stat.max_x;
    stat.min_y = gpu_stat.min_y;
    stat.max_y = gpu_stat.max_y;
    stat.min_z = gpu_stat.min_z;
    stat.max_z = gpu_stat.max_z;
    stat.num_points = gpu_stat.num_points;
    cluster_stats.push_back(stat);
  }

  // ========== STEP 6: Apply filtering and merging (using GPU-computed stats) ==========
  ProjectionUtils::filterClusterbyDensity(cluster_stats, cluster_indices, 
                                          density_weight_, size_weight_, 
                                          distance_weight_, score_threshold_);
  
  ProjectionUtils::mergeClusters(cluster_indices, filtered_point_cloud_, 
                                 cluster_stats, merge_threshold_);

  // ========== STEP 7: Recompute stats after merging ==========
  cluster_stats = ProjectionUtils::computeClusterStats(filtered_point_cloud_, cluster_indices);

  RCLCPP_INFO(get_logger(), "GPU clustering complete: %zu clusters found (from %d input points)", 
              cluster_indices.size(), N);
  RCLCPP_DEBUG(get_logger(), "GPU cluster stats computed: %zu clusters, %zu labels mapped",
               cluster_stats.size(), labels.size());

  return cluster_stats;
}

DetectionOutputs spatial_association::processDetections(
    const vision_msgs::msg::Detection2DArray &detection, 
    const geometry_msgs::msg::TransformStamped &transform,
    const std::array<double, 12> &projection_matrix,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const std::vector<ProjectionUtils::ClusterStats>& cluster_stats) {

  DetectionOutputs detection_outputs;

  if (!filtered_point_cloud_ || filtered_point_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Non-ground cloud data not available or empty");
    return detection_outputs;
  }

  if (cluster_indices.empty()) {
    return detection_outputs;
  }

  // Safety check: ensure cluster_stats matches cluster_indices size
  if (cluster_stats.size() != cluster_indices.size()) {
    RCLCPP_WARN(this->get_logger(), "cluster_stats size (%zu) != cluster_indices size (%zu), skipping",
                cluster_stats.size(), cluster_indices.size());
    return detection_outputs;
  }

  // Find the best matching cluster index (no mutation of cluster_indices)
  int best_cluster_idx = ProjectionUtils::computeBestClusterIndex(
      cluster_stats, cluster_indices, detection, transform,
      projection_matrix, object_detection_confidence_);

  // If no valid match found, return empty outputs
  if (best_cluster_idx < 0 || static_cast<size_t>(best_cluster_idx) >= cluster_indices.size()) {
    return detection_outputs;
  }

  // Safety check: ensure the selected cluster has valid indices
  const auto& selected_cluster = cluster_indices[best_cluster_idx];
  if (selected_cluster.indices.empty()) {
    RCLCPP_WARN(this->get_logger(), "Selected cluster at index %d has empty indices", best_cluster_idx);
    return detection_outputs;
  }

  // Create a single-element vector with the best cluster for downstream processing
  std::vector<pcl::PointIndices> best_cluster = {selected_cluster};

  // Precompute cluster boxes once to avoid recomputing in both computeBoundingBox and compute3DDetection
  auto boxes = ProjectionUtils::computeClusterBoxes(filtered_point_cloud_, best_cluster);

  if (publish_visualization_) {
    detection_outputs.bboxes = ProjectionUtils::computeBoundingBox(
        boxes, best_cluster, latest_lidar_msg_);
  }

  detection_outputs.detections3d = ProjectionUtils::compute3DDetection(boxes, best_cluster, latest_lidar_msg_);

  // VISUALIZATIONS
  // ----------------------------------------------------------------------------
  
  // assign colors to the clusters 
  if (publish_visualization_) {
    // Safety check: ensure working objects are initialized
    if (!working_colored_cluster_) {
      RCLCPP_WARN(this->get_logger(), "working_colored_cluster_ is null, skipping visualization");
    } else {
      detection_outputs.colored_cluster = working_colored_cluster_;
      ProjectionUtils::assignClusterColors(filtered_point_cloud_, best_cluster, detection_outputs.colored_cluster);
    }

    if (!working_centroid_cloud_) {
      RCLCPP_WARN(this->get_logger(), "working_centroid_cloud_ is null, skipping visualization");
    } else {
      detection_outputs.centroid_cloud = working_centroid_cloud_;
      detection_outputs.centroid_cloud->clear(); // Clear previous data
      for (auto &ci : best_cluster) {
        pcl::PointXYZ c;
        ProjectionUtils::computeClusterCentroid(filtered_point_cloud_, ci, c);
        detection_outputs.centroid_cloud->points.push_back(c);
      }
    }
  }
  return detection_outputs;
}


void spatial_association::multiDetectionsCallback(
    camera_object_detection_msgs::msg::BatchDetection::SharedPtr msg)
{
  // Safety check: ensure message is valid
  if (!msg) {
    RCLCPP_WARN(get_logger(), "Received null BatchDetection message");
    return;
  }

  if (camInfoMap_.size() < 3) {
    RCLCPP_WARN(get_logger(),
                "Waiting for 3 CameraInfo, have %zu", camInfoMap_.size());
    return;
  }

  // Check if we have point cloud data before processing detections
  if (!filtered_point_cloud_ || filtered_point_cloud_->empty()) {
    RCLCPP_WARN(get_logger(), "No non-ground cloud data available, skipping detection processing");
    return;
  }

  // Safety check: ensure tf_buffer_ is valid
  if (!tf_buffer_) {
    RCLCPP_ERROR(get_logger(), "TF buffer is null!");
    return;
  }

  // Safety check: ensure message has detections
  if (msg->detections.empty()) {
    RCLCPP_DEBUG(get_logger(), "BatchDetection message has no detections, skipping");
    return;
  }

  // PERFORM CLUSTERING ONCE (before camera loop) - significant performance optimization
  // This avoids redundant clustering computations for each camera
  // Also compute cluster stats once to avoid recomputing per camera
  RCLCPP_DEBUG(get_logger(), "Starting clustering for %zu detections", msg->detections.size());
  std::vector<ProjectionUtils::ClusterStats> cluster_stats;
  try {
    cluster_stats = performClustering(cluster_indices);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Exception in performClustering: %s", e.what());
    return;
  }
  
  if (cluster_indices.empty()) {
    RCLCPP_WARN(get_logger(), "No clusters found after filtering");
    return;
  }
  
  RCLCPP_DEBUG(get_logger(), "Clustering complete: %zu clusters, %zu stats", 
               cluster_indices.size(), cluster_stats.size());

  // Safety check: ensure cluster_stats matches cluster_indices size
  if (cluster_stats.size() != cluster_indices.size()) {
    RCLCPP_WARN(get_logger(), "cluster_stats size (%zu) != cluster_indices size (%zu) after clustering, skipping",
                cluster_stats.size(), cluster_indices.size());
    return;
  }

  // Prepare accumulators
  visualization_msgs::msg::MarkerArray combined_bboxes;
  vision_msgs::msg::Detection3DArray combined_detections3d;
  combined_detections3d.header = latest_lidar_msg_.header;

  pcl::PointCloud<pcl::PointXYZRGB> merged_cluster_cloud;
  pcl::PointCloud<pcl::PointXYZ> merged_centroid_cloud;
  int marker_id_offset = 0;

  // Loop over each camera's batch
  for (const auto &camera_batch : msg->detections) {
    try {
    // 1) Find camera info
    auto it = camInfoMap_.find(camera_batch.header.frame_id);
    if (it == camInfoMap_.end()) {
      RCLCPP_WARN(get_logger(),
                  "No CameraInfo for '%s', skipping",
                  camera_batch.header.frame_id.c_str());
      continue;
    }

    // Safety check: ensure CameraInfo is valid
    if (!it->second) {
      RCLCPP_WARN(get_logger(),
                  "CameraInfo for '%s' is null, skipping",
                  camera_batch.header.frame_id.c_str());
      continue;
    }

    // 2) Lookup TF from LiDAR → this camera
    geometry_msgs::msg::TransformStamped tf_cam_to_lidar;
    try {
      tf_cam_to_lidar = tf_buffer_->lookupTransform(
          camera_batch.header.frame_id,
          lidar_frame_,
          tf2::TimePointZero);
    } catch (tf2::TransformException &e) {
      RCLCPP_WARN(get_logger(),
                  "TF %s→%s failed: %s",
                  lidar_frame_.c_str(),
                  camera_batch.header.frame_id.c_str(),
                  e.what());
      continue;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(),
                   "Unexpected exception during TF lookup: %s",
                   e.what());
      continue;
    }

    // 3) Run 3D detection pipeline using pre-computed clusters and stats
    auto detection_results = processDetections(
        camera_batch,
        tf_cam_to_lidar,
        it->second->p,
        cluster_indices,
        cluster_stats);

    // 4) Collect MarkerArray
    for (auto &marker : detection_results.bboxes.markers) {
      marker.ns = camera_batch.header.frame_id;
      marker.id += marker_id_offset;
      combined_bboxes.markers.push_back(marker);
    }
    marker_id_offset += static_cast<int>(detection_results.bboxes.markers.size());

    // 5) Collect Detection3DArray
    for (auto &det : detection_results.detections3d.detections) {
      combined_detections3d.detections.push_back(det);
    }

    // 6) Merge cluster-cloud visuals if requested
    if (publish_visualization_) {
      if (detection_results.colored_cluster) {
        merged_cluster_cloud += *detection_results.colored_cluster;
      }
      if (detection_results.centroid_cloud) {
        merged_centroid_cloud += *detection_results.centroid_cloud;
      }
    }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "Exception processing camera batch '%s': %s",
                   camera_batch.header.frame_id.c_str(), e.what());
      continue;
    }
  }

  // 7) Publish visualization data only if enabled and publishers exist
  if (publish_visualization_ && bounding_box_pub_) {
    bounding_box_pub_->publish(combined_bboxes);
  }

  if (publish_visualization_ && filtered_lidar_pub_) {
    sensor_msgs::msg::PointCloud2 pcl2_msg;
    pcl::toROSMsg(merged_cluster_cloud, pcl2_msg);
    pcl2_msg.header = latest_lidar_msg_.header;
    filtered_lidar_pub_->publish(pcl2_msg);
  }

  if (publish_visualization_ && cluster_centroid_pub_) {
    sensor_msgs::msg::PointCloud2 pcl2_msg;
    pcl::toROSMsg(merged_centroid_cloud, pcl2_msg);
    pcl2_msg.header = latest_lidar_msg_.header;
    cluster_centroid_pub_->publish(pcl2_msg);
  }

  // 8) Always publish the fused Detection3DArray
  combined_detections3d.header.stamp = this->get_clock()->now();
  detection_3d_pub_->publish(combined_detections3d);
}

void spatial_association::lidarClusteringTimerCallback() {
  // Only process if we have point cloud data
  if (!filtered_point_cloud_ || filtered_point_cloud_->empty()) {
    return;
  }

  // Perform clustering
  std::vector<pcl::PointIndices> local_cluster_indices;
  std::vector<ProjectionUtils::ClusterStats> cluster_stats;
  
  try {
    RCLCPP_INFO(get_logger(), "Timer: Starting clustering for %zu points", 
                filtered_point_cloud_->size());
    cluster_stats = performClustering(local_cluster_indices);
    
    if (local_cluster_indices.empty()) {
      RCLCPP_DEBUG(get_logger(), "Timer: No clusters found");
      return;
    }
    
    RCLCPP_INFO(get_logger(), "Timer: Found %zu clusters, publishing bounding boxes", 
                local_cluster_indices.size());
    
    // Publish bounding boxes for all clusters
    publishAllClusterBoxes(local_cluster_indices, cluster_stats);
    
  } catch (const std::exception& e) {
    RCLCPP_ERROR(get_logger(), "Timer: Exception in clustering: %s", e.what());
  }
}

void spatial_association::publishAllClusterBoxes(
    const std::vector<pcl::PointIndices>& cluster_indices,
    const std::vector<ProjectionUtils::ClusterStats>& cluster_stats) {
  
  if (cluster_indices.empty()) {
    return;
  }

  // Safety check: ensure cluster_stats matches cluster_indices size
  if (cluster_stats.size() != cluster_indices.size()) {
    RCLCPP_WARN(get_logger(), "cluster_stats size (%zu) != cluster_indices size (%zu), skipping publish",
                cluster_stats.size(), cluster_indices.size());
    return;
  }

  // Compute bounding boxes for all clusters
  auto boxes = ProjectionUtils::computeClusterBoxes(filtered_point_cloud_, cluster_indices);
  
  // Create MarkerArray for visualization
  if (publish_visualization_ && bounding_box_pub_) {
    auto marker_array = ProjectionUtils::computeBoundingBox(boxes, cluster_indices, latest_lidar_msg_);
    bounding_box_pub_->publish(marker_array);
    RCLCPP_INFO(get_logger(), "Published %zu bounding box markers", marker_array.markers.size());
  }

  // Create Detection3DArray
  if (detection_3d_pub_) {
    auto detections3d = ProjectionUtils::compute3DDetection(boxes, cluster_indices, latest_lidar_msg_);
    detections3d.header.stamp = this->get_clock()->now();
    detection_3d_pub_->publish(detections3d);
    RCLCPP_INFO(get_logger(), "Published %zu 3D detections", detections3d.detections.size());
  }

  // Publish visualization point clouds if enabled
  if (publish_visualization_) {
    if (filtered_lidar_pub_ && working_colored_cluster_) {
      working_colored_cluster_->clear();
      ProjectionUtils::assignClusterColors(filtered_point_cloud_, cluster_indices, working_colored_cluster_);
      sensor_msgs::msg::PointCloud2 pcl2_msg;
      pcl::toROSMsg(*working_colored_cluster_, pcl2_msg);
      pcl2_msg.header = latest_lidar_msg_.header;
      filtered_lidar_pub_->publish(pcl2_msg);
    }

    if (cluster_centroid_pub_ && working_centroid_cloud_) {
      working_centroid_cloud_->clear();
      for (const auto& ci : cluster_indices) {
        pcl::PointXYZ c;
        ProjectionUtils::computeClusterCentroid(filtered_point_cloud_, ci, c);
        working_centroid_cloud_->points.push_back(c);
      }
      sensor_msgs::msg::PointCloud2 pcl2_msg;
      pcl::toROSMsg(*working_centroid_cloud_, pcl2_msg);
      pcl2_msg.header = latest_lidar_msg_.header;
      cluster_centroid_pub_->publish(pcl2_msg);
    }
  }
}


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spatial_association>());
  rclcpp::shutdown();
  return 0;
}
