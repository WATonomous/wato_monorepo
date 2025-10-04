#pragma once

#include <memory>
#include <string>

#include <Eigen/Core>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/header.hpp>

#include "ground_removal_core.hpp"

namespace wato::percpetion::patchworkpp {

class GroundRemovalNode : public rclcpp::Node {
 public:
  GroundRemovalNode() = delete;
  explicit GroundRemovalNode(const rclcpp::NodeOptions &options);

 private:
  void declareParameters(patchwork::Params &params);
  void removeGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);
  void publishFilteredCloud(const Eigen::MatrixX3f &nonground_points,
                            const std_msgs::msg::Header &header);
  void publishDebugClouds(const Eigen::MatrixX3f &est_ground,
                          const Eigen::MatrixX3f &est_nonground,
                          const std_msgs::msg::Header &header);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr filtered_cloud_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ground_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr nonground_publisher_;

  std::unique_ptr<GroundRemovalCore> core_;

  bool publish_debug_{false};
  bool publish_original_{false};

  std::string cloud_topic_{"/LIDAR_TOP"};
  std::string filtered_topic_{"/patchworkpp/filtered_cloud"};
  std::string ground_topic_{"/patchworkpp/ground_cloud"};
  std::string nonground_topic_{"/patchworkpp/non_ground_cloud"};
};

}  // namespace wato::percpetion::patchworkpp
