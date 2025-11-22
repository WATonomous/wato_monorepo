#include "spatial_association.hpp"

spatial_association::spatial_association() : Node("spatial_association") {
  initializeParams();

  // Initialize core clustering library
  core_ = std::make_unique<SpatialAssociationCore>();
  
  // Initialize working PCL objects for visualization
  working_colored_cluster_.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
  working_centroid_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  
  // Set core parameters from ROS parameters
  SpatialAssociationCore::ClusteringParams core_params;
  core_params.voxel_size = voxel_size_;
  core_params.euclid_cluster_tolerance = euclid_cluster_tolerance_;
  core_params.euclid_min_cluster_size = euclid_min_cluster_size_;
  core_params.euclid_max_cluster_size = euclid_max_cluster_size_;
  core_params.use_adaptive_clustering = use_adaptive_clustering_;
  core_params.euclid_close_threshold = euclid_close_threshold_;
  core_params.euclid_close_tolerance_mult = euclid_close_tolerance_mult_;
  core_params.density_weight = density_weight_;
  core_params.size_weight = size_weight_;
  core_params.distance_weight = distance_weight_;
  core_params.score_threshold = score_threshold_;
  core_params.merge_threshold = merge_threshold_;
  core_params.object_detection_confidence = object_detection_confidence_;
  core_->setParams(core_params);

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
      this->create_publisher<vision_msgs::msg::Detection3DArray>("/detection_3d", 10);
  bounding_box_pub_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/bounding_box", 10);  

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
  

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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

  this->declare_parameter<std::string>("lidar_top_frame", "LIDAR_TOP");

  this->declare_parameter<bool>("publish_visualization", true);
  this->declare_parameter<bool>("debug_logging", false);

  // Voxel downsampling parameters
  this->declare_parameter<float>("voxel_size", 0.2f);  // Fixed voxel size (meters)


  // Euclidean Clustering Parameters
  this->declare_parameter<double>("euclid_params.cluster_tolerance", 0.5);
  this->declare_parameter<int>("euclid_params.min_cluster_size", 50);
  this->declare_parameter<int>("euclid_params.max_cluster_size", 700);
  this->declare_parameter<bool>("euclid_params.use_adaptive_clustering", true);
  this->declare_parameter<double>("euclid_params.close_threshold", 10.0);
  this->declare_parameter<double>("euclid_params.close_tolerance_mult", 1.5);

  // Density Filtering Parameters
  this->declare_parameter<double>("density_filter_params.density_weight", 0.6);
  this->declare_parameter<double>("density_filter_params.size_weight", 0.8);
  this->declare_parameter<double>("density_filter_params.distance_weight", 0.7);
  this->declare_parameter<double>("density_filter_params.score_threshold", 0.6);

  this->declare_parameter<double>("merge_threshold", 0.3);

  this->declare_parameter<float>("object_detection_confidence", 0.4);

  // Get parameters
  publish_visualization_ = this->get_parameter("publish_visualization").as_bool();

  // Get voxel downsampling parameter
  voxel_size_ = static_cast<float>(this->get_parameter("voxel_size").as_double());
  // Note: voxel_filter_ is now managed by the core library and set via core_->setParams()

  camera_info_topic_front_ = this->get_parameter("camera_info_topic_front_").as_string();
  camera_info_topic_right_ = this->get_parameter("camera_info_topic_right_").as_string();
  camera_info_topic_left_ = this->get_parameter("camera_info_topic_left_").as_string();

  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  non_ground_cloud_topic_ = this->get_parameter("non_ground_cloud_topic").as_string();
  detections_topic_ = this->get_parameter("detections_topic").as_string();

  filtered_lidar_topic_ = this->get_parameter("filtered_lidar_topic").as_string();
  cluster_centroid_topic_ = this->get_parameter("cluster_centroid_topic").as_string();
  bounding_box_topic_ = this->get_parameter("bounding_box_topic").as_string();

  lidar_frame_ = this->get_parameter("lidar_top_frame").as_string();


  euclid_cluster_tolerance_ = this->get_parameter("euclid_params.cluster_tolerance").as_double();
  euclid_min_cluster_size_ = this->get_parameter("euclid_params.min_cluster_size").as_int();
  euclid_max_cluster_size_ = this->get_parameter("euclid_params.max_cluster_size").as_int();
  use_adaptive_clustering_ = this->get_parameter("euclid_params.use_adaptive_clustering").as_bool();
  euclid_close_threshold_ = this->get_parameter("euclid_params.close_threshold").as_double();
  euclid_close_tolerance_mult_ = this->get_parameter("euclid_params.close_tolerance_mult").as_double();

  density_weight_ = this->get_parameter("density_filter_params.density_weight").as_double();
  size_weight_ = this->get_parameter("density_filter_params.size_weight").as_double();
  distance_weight_ = this->get_parameter("density_filter_params.distance_weight").as_double();
  score_threshold_ = this->get_parameter("density_filter_params.score_threshold").as_double();

  merge_threshold_ = this->get_parameter("merge_threshold").as_double();

  object_detection_confidence_ = this->get_parameter("object_detection_confidence").as_double();

  debug_logging_ = this->get_parameter("debug_logging").as_bool();

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
  
  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(latest_lidar_msg_, *input_cloud);

  // Use core library to process point cloud (downsampling)
  filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  core_->processPointCloud(input_cloud, filtered_point_cloud_);
}

void spatial_association::nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Received non-ground cloud with %d points", msg->width * msg->height);
  }
  
  // Store the non-ground cloud message from patchwork
  latest_lidar_msg_ = *msg;
  
  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(latest_lidar_msg_, *input_cloud);

  if (input_cloud->empty()) {
    RCLCPP_WARN(this->get_logger(), "Received empty non-ground cloud");
    filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
    return;
  }

  // Use core library to process point cloud (downsampling)
  filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
  core_->processPointCloud(input_cloud, filtered_point_cloud_);
 
  if (debug_logging_) {
    RCLCPP_INFO(this->get_logger(), "Processed non-ground cloud: %zu points after downsampling", filtered_point_cloud_->size());
  }

}


DetectionOutputs spatial_association::processDetections(const vision_msgs::msg::Detection2DArray &detection, 
                                  const geometry_msgs::msg::TransformStamped &transform,
                                  const std::array<double, 12> &projection_matrix,
                                  const std::vector<pcl::PointIndices>& cluster_indices_input) {

  DetectionOutputs detection_outputs;

  if (!filtered_point_cloud_ || filtered_point_cloud_->empty()) {
    RCLCPP_WARN(this->get_logger(), "Non-ground cloud data not available or empty");
    return detection_outputs;
  }

  if (cluster_indices_input.empty()) {
    return detection_outputs;
  }

  // Create a copy of cluster indices for IOU filtering (this modifies the vector)
  std::vector<pcl::PointIndices> cluster_indices = cluster_indices_input;

  // Precompute cluster stats once to avoid redundant point scans
  auto cluster_stats = ProjectionUtils::computeClusterStats(filtered_point_cloud_, cluster_indices);

  // iou score between x and y area of clusters in camera plane and detections
  ProjectionUtils::computeHighestIOUCluster(cluster_stats, cluster_indices, detection, transform,
                                            projection_matrix, object_detection_confidence_);

  // Precompute cluster boxes once to avoid recomputing in both computeBoundingBox and compute3DDetection
  // Use core library to compute boxes
  auto boxes = core_->computeClusterBoxes(filtered_point_cloud_, cluster_indices);

  if (publish_visualization_) {
    detection_outputs.bboxes = ProjectionUtils::computeBoundingBox(
        boxes, cluster_indices, latest_lidar_msg_);
  }

  detection_outputs.detections3d = ProjectionUtils::compute3DDetection(boxes, cluster_indices, latest_lidar_msg_);

  // VISUALIZATIONS
  // ----------------------------------------------------------------------------
  
  // assign colors to the clusters using core library
  if (publish_visualization_) {
    detection_outputs.colored_cluster = working_colored_cluster_;
    core_->assignClusterColors(filtered_point_cloud_, cluster_indices, detection_outputs.colored_cluster);

    detection_outputs.centroid_cloud = working_centroid_cloud_;
    core_->computeClusterCentroids(filtered_point_cloud_, cluster_indices, detection_outputs.centroid_cloud);
  }
  return detection_outputs;
}


void spatial_association::multiDetectionsCallback(
    camera_object_detection_msgs::msg::BatchDetection::SharedPtr msg)
{
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

  // PERFORM CLUSTERING ONCE (before camera loop) - significant performance optimization
  // This avoids redundant clustering computations for each camera
  // Use core library for clustering
  core_->performClustering(filtered_point_cloud_, cluster_indices);
  
  if (cluster_indices.empty()) {
    RCLCPP_DEBUG(get_logger(), "No clusters found after filtering");
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
    // 1) Find camera info
    auto it = camInfoMap_.find(camera_batch.header.frame_id);
    if (it == camInfoMap_.end()) {
      RCLCPP_WARN(get_logger(),
                  "No CameraInfo for '%s', skipping",
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
    }

    // 3) Run 3D detection pipeline using pre-computed clusters
    auto detection_results = processDetections(
        camera_batch,
        tf_cam_to_lidar,
        it->second->p,
        cluster_indices);

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
      merged_cluster_cloud += *detection_results.colored_cluster;
      merged_centroid_cloud += *detection_results.centroid_cloud;
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


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<spatial_association>());
  rclcpp::shutdown();
  return 0;
}
