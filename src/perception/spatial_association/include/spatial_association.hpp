#ifndef SPATIAL_ASSOCIATION_HPP
#define SPATIAL_ASSOCIATION_HPP

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <camera_object_detection_msgs/msg/batch_detection.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "projection_utils.hpp"

#include <unordered_map>

struct DetectionOutputs {
  visualization_msgs::msg::MarkerArray bboxes;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster;
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud;
  vision_msgs::msg::Detection3DArray detections3d;
};

class spatial_association : public rclcpp::Node {
 public:
  spatial_association();

 private:
  // CONFIG/VISUALIZATION
  bool publish_visualization_;

  // CAMERA
  // ------------------------------------------------------------------------------------------------------

  std::unordered_map<std::string, sensor_msgs::msg::CameraInfo::SharedPtr> camInfoMap_;
  void multiCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // LIDAR
  // -------------------------------------------------------------------------------------------------------
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
  void nonGroundCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  sensor_msgs::msg::PointCloud2 latest_lidar_msg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud_;
  std::vector<pcl::PointIndices> cluster_indices;

  // Working PCL objects for performance optimization
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_downsampled_cloud_;
  pcl::VoxelGrid<pcl::PointXYZ> voxel_filter_;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr working_colored_cluster_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr working_centroid_cloud_;

  // DETECTIONS
  // ------------------------------------------------------------------------------------------------------
  void multiDetectionsCallback(camera_object_detection_msgs::msg::BatchDetection::SharedPtr msg);

  // Perform clustering once (shared across all cameras)
  void performClustering(std::vector<pcl::PointIndices>& cluster_indices);
  
  DetectionOutputs processDetections(
      const vision_msgs::msg::Detection2DArray& detections,
      const geometry_msgs::msg::TransformStamped& transform,
      const std::array<double, 12>& projection_matrix,
      const std::vector<pcl::PointIndices>& cluster_indices);

  geometry_msgs::msg::TransformStamped transform;

  // SUBSCRIBERS
  // -----------------------------------------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr non_ground_cloud_sub_;
  rclcpp::Subscription<camera_object_detection_msgs::msg::BatchDetection>::SharedPtr batch_dets_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_front_,camera_info_sub_left_, camera_info_sub_right_;


  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // PUBLISHERS
  // ------------------------------------------------------------------------------------------------------
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_centroid_pub_;
  
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_pub_;
  rclcpp::Publisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_3d_pub_;

  // FUNCTION PARAMS
  // -------------------------------------------------------------------------------------------------

  void initializeParams();

  std::string camera_info_topic_front_;
  std::string camera_info_topic_left_;
  std::string camera_info_topic_right_;

  std::string lidar_topic_;
  std::string non_ground_cloud_topic_;
  std::string detections_topic_;

  std::string filtered_lidar_topic_;
  std::string cluster_centroid_topic_;
  std::string bounding_box_topic_;

  std::string lidar_frame_;

  // Filtering parameters

  double euclid_cluster_tolerance_;
  int euclid_min_cluster_size_;
  int euclid_max_cluster_size_;

  double density_weight_;
  double size_weight_;
  double distance_weight_;
  double score_threshold_;

  double merge_threshold_;

  float object_detection_confidence_;

};

#endif
