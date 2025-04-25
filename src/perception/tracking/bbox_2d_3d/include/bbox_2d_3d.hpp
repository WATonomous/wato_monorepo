#ifndef BBOX_2D_3D_HPP
#define BBOX_2D_3D_HPP

#include <cv_bridge/cv_bridge.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <camera_object_detection_msgs/msg/batch_detection.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include "projection_utils.hpp"

#include <unordered_map>

struct DetectionOutputs {
  visualization_msgs::msg::MarkerArray bboxes;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cluster;
  pcl::PointCloud<pcl::PointXYZ>::Ptr centroid_cloud;
};

class bbox_2d_3d : public rclcpp::Node {
 public:
  bbox_2d_3d();

 private:
  // CAMERA
  // ------------------------------------------------------------------------------------------------------

  std::unordered_map<std::string, sensor_msgs::msg::CameraInfo::SharedPtr> camInfoMap_;
  void multiCameraInfoCallback(const sensor_msgs::msg::CameraInfo::SharedPtr msg);

  // LIDAR
  // -------------------------------------------------------------------------------------------------------
  void lidarCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

  sensor_msgs::msg::PointCloud2 latest_lidar_msg_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_point_cloud_;
  std::vector<pcl::PointIndices> cluster_indices;

  // DETECTIONS
  // ------------------------------------------------------------------------------------------------------
  void multiDetectionsCallback(camera_object_detection_msgs::msg::BatchDetection::SharedPtr msg);

  DetectionOutputs processDetections(
      const vision_msgs::msg::Detection2DArray& detections,
      const geometry_msgs::msg::TransformStamped& transform,
      const std::array<double, 12>& projection_matrix);

  geometry_msgs::msg::TransformStamped transform;

  // SUBSCRIBERS
  // -----------------------------------------------------------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;
  rclcpp::Subscription<camera_object_detection_msgs::msg::BatchDetection>::SharedPtr batch_dets_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_front_,camera_info_sub_left_, camera_info_sub_right_;

  // for visualization
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // PUBLISHERS
  // ------------------------------------------------------------------------------------------------------
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_lidar_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr colored_cluster_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cluster_centroid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bounding_box_pub_;

  // FUNCTION PARAMS
  // -------------------------------------------------------------------------------------------------

  void initializeParams();

  std::string camera_info_topic_front_;
  std::string camera_info_topic_left_;
  std::string camera_info_topic_right_;

  std::string lidar_topic_;
  std::string detections_topic_;

  std::string filtered_lidar_topic_;
  std::string cluster_centroid_topic_;
  std::string bounding_box_topic_;

  std::string lidar_frame_;

  // Filtering parameters
  double ransac_distance_threshold_;
  int ransac_max_iterations_;

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