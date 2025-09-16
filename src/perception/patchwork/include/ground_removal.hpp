// Patchwork++
#include "patchwork/patchworkpp.h"


#include <string>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

namespace patchworkpp_ros {

class GroundRemovalServer : public rclcpp::Node {
 public:
  /// GroundRemovalServer constructor
  GroundRemovalServer() = delete;
  explicit GroundRemovalServer(const rclcpp::NodeOptions &options);

 private:
  /// Process incoming point cloud and remove ground
  void removeGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

  /// Publish the filtered point cloud (ground removed)
  void publishFilteredCloud(const Eigen::MatrixX3f &nonground_points,
                           const std_msgs::msg::Header header_msg);

  /// Publish additional debug information if enabled
  void publishDebugClouds(const Eigen::MatrixX3f &est_ground,
                         const Eigen::MatrixX3f &est_nonground,
                         const std_msgs::msg::Header header_msg);

 private:
  /// Data subscribers.
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  /// Data publishers.
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
  
  /// Debug publishers (option)
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

  /// Patchwork++
  std::unique_ptr<patchwork::PatchWorkpp> Patchworkpp_;

  /// Configuration parameters
  std::string base_frame_{"base_link"};
  bool publish_debug_{false};
};

}  
