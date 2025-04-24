#include "bbox_2d_3d.hpp"

bbox_2d_3d::bbox_2d_3d() : Node("bbox_2d_3d") {
  initializeParams();

  // SUBSCRIBERS
  // -------------------------------------------------------------------------------------------------

  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_, 10, std::bind(&bbox_2d_3d::lidarCallback, this, std::placeholders::_1));

  batch_dets_sub_ = this->create_subscription<camera_object_detection_msgs::msg::BatchDetection>(
      detections_topic_, 10, std::bind(&bbox_2d_3d::multiDetectionsCallback, this, std::placeholders::_1));

  // image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
  //     "/annotated_img", 10, std::bind(&bbox_2d_3d::imageCallback, this, std::placeholders::_1));

  auto info_qos = rclcpp::SensorDataQoS();

  camera_info_sub_front_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_front_, info_qos,
        std::bind(&bbox_2d_3d::multiCameraInfoCallback, this, std::placeholders::_1));

  camera_info_sub_left_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_left_, info_qos,
        std::bind(&bbox_2d_3d::multiCameraInfoCallback, this, std::placeholders::_1));
    
  camera_info_sub_right_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_right_, info_qos,
        std::bind(&bbox_2d_3d::multiCameraInfoCallback, this, std::placeholders::_1));

  // PUBLISHERS
  // --------------------------------------------------------------------------------------------------

  filtered_lidar_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(filtered_lidar_topic_, 10);
  cluster_centroid_pub_ =
      this->create_publisher<sensor_msgs::msg::PointCloud2>(cluster_centroid_topic_, 10);
  bounding_box_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(bounding_box_topic_, 10);

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void bbox_2d_3d::initializeParams() {

    this->declare_parameter<std::string>("camera_info_topic_front_", "/CAM_FRONT/camera_info");
    this->declare_parameter<std::string>("camera_info_topic_left_", "/CAM_FRONT_LEFT/camera_info");
    this->declare_parameter<std::string>("camera_info_topic_right_", "/CAM_FRONT_RIGHT/camera_info");

    this->declare_parameter<std::string>("lidar_topic", "/LIDAR_TOP");
    this->declare_parameter<std::string>("detections_topic", "/batched_camera_message");

  this->declare_parameter<std::string>("filtered_lidar_topic", "/filtered_lidar");
  this->declare_parameter<std::string>("cluster_centroid_topic", "/cluster_centroid");
  this->declare_parameter<std::string>("bounding_box_topic", "/bounding_box");

  this->declare_parameter<std::string>("lidar_top_frame", "LIDAR_TOP");

  // RANSAC Parameters
  this->declare_parameter<double>("ransac_params.distance_threshold", 0.5);
  this->declare_parameter<int>("ransac_params.max_iterations", 1500);

  // Euclidean Clustering Parameters
  this->declare_parameter<double>("euclid_params.cluster_tolerance", 1.2);
  this->declare_parameter<int>("euclid_params.min_cluster_size", 30);
  this->declare_parameter<int>("euclid_params.max_cluster_size", 700);

  // Density Filtering Parameters
  this->declare_parameter<double>("density_filter_params.density_weight", 0.6);
  this->declare_parameter<double>("density_filter_params.size_weight", 0.8);
  this->declare_parameter<double>("density_filter_params.distance_weight", 0.7);
  this->declare_parameter<double>("density_filter_params.score_threshold", 0.5);

  this->declare_parameter<double>("merge_threshold", 0.4);

  this->declare_parameter<float>("object_detection_confidence", 0.3);

  // Get parameters
  camera_info_topic_front_ = this->get_parameter("camera_info_topic_front_").as_string();
  camera_info_topic_right_ = this->get_parameter("camera_info_topic_right_").as_string();
  camera_info_topic_left_ = this->get_parameter("camera_info_topic_left_").as_string();

  lidar_topic_ = this->get_parameter("lidar_topic").as_string();
  detections_topic_ = this->get_parameter("detections_topic").as_string();

  filtered_lidar_topic_ = this->get_parameter("filtered_lidar_topic").as_string();
  cluster_centroid_topic_ = this->get_parameter("cluster_centroid_topic").as_string();
  bounding_box_topic_ = this->get_parameter("bounding_box_topic").as_string();

  lidar_frame_ = this->get_parameter("lidar_top_frame").as_string();

  ransac_distance_threshold_ = this->get_parameter("ransac_params.distance_threshold").as_double();
  ransac_max_iterations_ = this->get_parameter("ransac_params.max_iterations").as_int();

  euclid_cluster_tolerance_ = this->get_parameter("euclid_params.cluster_tolerance").as_double();
  euclid_min_cluster_size_ = this->get_parameter("euclid_params.min_cluster_size").as_int();
  euclid_max_cluster_size_ = this->get_parameter("euclid_params.max_cluster_size").as_int();

  density_weight_ = this->get_parameter("density_filter_params.density_weight").as_double();
  size_weight_ = this->get_parameter("density_filter_params.size_weight").as_double();
  distance_weight_ = this->get_parameter("density_filter_params.distance_weight").as_double();
  score_threshold_ = this->get_parameter("density_filter_params.score_threshold").as_double();

  merge_threshold_ = this->get_parameter("merge_threshold").as_double();

  object_detection_confidence_ = this->get_parameter("object_detection_confidence").as_double();
  RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}

void bbox_2d_3d::multiCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  const auto frame = msg->header.frame_id;
  camInfoMap_[frame] = msg;
  // RCLCPP_INFO(get_logger(),
  //   "cached CameraInfo for '%s'  (now have %zu cameras)",
  //   frame.c_str(), camInfoMap_.size());
}

// void bbox_2d_3d::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
//   try {
//     image_data_ = cv_bridge::toCvCopy(msg, "bgr8");
//   } catch (cv_bridge::Exception& e) {
//     RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
//     return;
//   }
// }

void bbox_2d_3d::lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  // store the lidar msg
  latest_lidar_msg_ = *msg;
  filtered_point_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

  // Convert to PCL
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(latest_lidar_msg_, *point_cloud);

  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel;
  voxel.setInputCloud(point_cloud);
  voxel.setLeafSize(0.1f, 0.1f, 0.1f);
  voxel.filter(*downsampled_cloud);

  ProjectionUtils::removeGroundPlane(downsampled_cloud, ransac_distance_threshold_, ransac_max_iterations_);
  filtered_point_cloud_ = downsampled_cloud;

  // Remove outliers from the LiDAR point cloud
  /*  int meanK = 30; // Number of neighbors to analyze for each point
      double stddevMulThresh = 1.5; // Standard deviation multiplier threshold

      ProjectionUtils::removeOutliers(filtered_point_cloud_, meanK, stddevMulThresh); */
}

void bbox_2d_3d::processDetections(const vision_msgs::msg::Detection2DArray &detection, 
                                  const geometry_msgs::msg::TransformStamped &transform,
                                  const std::array<double, 12> &projection_matrix) {

  // checks for if pointers available, prevents accessing null pointers

  if (!filtered_point_cloud_) {
    RCLCPP_WARN(this->get_logger(), "Lidar data not available");
    return;
  }

  // CLUSTERING-----------------------------------------------------------------------------------------------

  // Perform Euclidean clustering, populate cluster_indices
  std::vector<pcl::PointIndices> cluster_indices;
  ProjectionUtils::euclideanClusterExtraction(filtered_point_cloud_, euclid_cluster_tolerance_,
                                              euclid_min_cluster_size_, euclid_max_cluster_size_,
                                              cluster_indices);

  // filter clusters by density, size and distance
  ProjectionUtils::filterClusterbyDensity(filtered_point_cloud_, cluster_indices, density_weight_,
                                          size_weight_, distance_weight_, score_threshold_);

  // merge clusters that are close to each other, determined through distance between their
  // centroids
  ProjectionUtils::mergeClusters(cluster_indices, filtered_point_cloud_, merge_threshold_);

  // calculate the best fitting iou score between the x and y area of the clusters in the camera
  // plane and the detections
  ProjectionUtils::computeHighestIOUCluster(filtered_point_cloud_, cluster_indices, detection, transform,
                                            projection_matrix, object_detection_confidence_);

  // assign colors to the clusters (strictly for visualiztion)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_clustered_cloud(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  ProjectionUtils::assignClusterColors(filtered_point_cloud_, cluster_indices,
                                       colored_clustered_cloud);

  // publish the messages
  sensor_msgs::msg::PointCloud2 filtered_lidar_msg;
  pcl::toROSMsg(*colored_clustered_cloud, filtered_lidar_msg);
  filtered_lidar_msg.header = latest_lidar_msg_.header;
  filtered_lidar_pub_->publish(filtered_lidar_msg);

  visualization_msgs::msg::MarkerArray bbox_msg;
  bbox_msg = ProjectionUtils::computeBoundingBox(filtered_point_cloud_, cluster_indices,
                                                 latest_lidar_msg_);
  bounding_box_pub_->publish(bbox_msg);

  // CENTROIDS (FOR VISUALIZATIONS)
  // ----------------------------------------------------------------------------

  pcl::PointCloud<pcl::PointXYZ> centroid_cloud;
  for (const auto& cluster : cluster_indices) {
    pcl::PointXYZ centroid;
    ProjectionUtils::computeClusterCentroid(filtered_point_cloud_, cluster, centroid);
    centroid_cloud.push_back(centroid);
  }

  sensor_msgs::msg::PointCloud2 centroid_msg;
  pcl::toROSMsg(centroid_cloud, centroid_msg);
  centroid_msg.header = latest_lidar_msg_.header;
  cluster_centroid_pub_->publish(centroid_msg);

  // IMAGE PROJECTION (FOR DEBUGGING)
  // ---------------------------------------------------------------------------

  // cv::Mat image = image_data_->image.clone();

  // for (const auto& point : filtered_point_cloud_->points) {
  //   auto projected_point = ProjectionUtils::projectLidarToCamera(transform, camInfo_->p, point);
  //   if (projected_point) {
  //     cv::circle(image, *projected_point, 2, cv::Scalar(0, 255, 0), -1);
  //   }
  // }

  //   try {
  //       transform = tf_buffer_->lookupTransform(msg->header.frame_id, lidar_frame_, tf2::TimePointZero);
  //   } catch (tf2::TransformException &ex) {
  //       RCLCPP_WARN(this->get_logger(), "Transform lookup failed: %s", ex.what());
  //       return;
  //   }

}

void bbox_2d_3d::multiDetectionsCallback(
  camera_object_detection_msgs::msg::BatchDetection::SharedPtr msg) {

  if (camInfoMap_.size() < 3) {
    RCLCPP_WARN(get_logger(),
      "Still waiting for all 3 CameraInfo (have %zu); skipping",
      camInfoMap_.size());
    return;
  }

  for (auto &dets : msg->detections) {

    const auto &cam_frame = dets.header.frame_id;
    // RCLCPP_INFO(get_logger(),
    //   "batch contains %zu boxes in frame '%s'",
    //   dets.detections.size(), cam_frame.c_str());

    // look up intrinsics
    auto it = camInfoMap_.find(cam_frame);
    if (it == camInfoMap_.end()) {
      RCLCPP_WARN(get_logger(),
        "No CameraInfo for frame '%s', skipping", cam_frame.c_str());
      continue;
    }
    const auto &cam_info = it->second;

    geometry_msgs::msg::TransformStamped transform;
    try {
      transform = tf_buffer_->lookupTransform(
        cam_frame,        
        lidar_frame_,     
        tf2::TimePointZero
      );
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(),
        "TF lookup %s → %s failed: %s",
        lidar_frame_.c_str(), cam_frame.c_str(), ex.what());
      continue;
    }

    // run your existing pipeline
    processDetections(
      dets,         
      transform,            
      cam_info->p 
    );
  }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<bbox_2d_3d>());
  rclcpp::shutdown();
  return 0;
}