#include <memory>
#include <utility>
#include <vector>

#include <Eigen/Core>

// Patchwork++-ROS
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>

#include "ground_removal.hpp"

namespace patchworkpp_ros {

// Utility functions for point cloud conversion
namespace utils {

sensor_msgs::msg::PointCloud2 EigenMatToPointCloud2(
    const Eigen::MatrixX3f &points, 
    const std_msgs::msg::Header &header) {
  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header = header;
  cloud_msg.height = 1;
  cloud_msg.width = points.rows();
  cloud_msg.is_bigendian = false;
  cloud_msg.is_dense = false;
  
  // Set fields
  sensor_msgs::msg::PointField field;
  field.name = "x";
  field.offset = 0;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud_msg.fields.push_back(field);
  
  field.name = "y";
  field.offset = 4;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud_msg.fields.push_back(field);
  
  field.name = "z";
  field.offset = 8;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count = 1;
  cloud_msg.fields.push_back(field);
  
  cloud_msg.point_step = 12; // 3 * 4 bytes
  cloud_msg.row_step = cloud_msg.point_step * cloud_msg.width;
  cloud_msg.data.resize(cloud_msg.row_step * cloud_msg.height);
  
  // Copy data
  memcpy(cloud_msg.data.data(), points.data(), cloud_msg.data.size());
  
  return cloud_msg;
}

Eigen::MatrixX3f PointCloud2ToEigenMat(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg) {
  Eigen::MatrixX3f points(cloud_msg->width, 3);
  
  // Find x, y, z field indices
  int x_idx = -1, y_idx = -1, z_idx = -1;
  for (size_t i = 0; i < cloud_msg->fields.size(); ++i) {
    if (cloud_msg->fields[i].name == "x") x_idx = i;
    else if (cloud_msg->fields[i].name == "y") y_idx = i;
    else if (cloud_msg->fields[i].name == "z") z_idx = i;
  }
  
  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    throw std::runtime_error("PointCloud2 missing x, y, or z field");
  }
  
  // Extract points
  for (size_t i = 0; i < cloud_msg->width; ++i) {
    size_t offset = i * cloud_msg->point_step;
    points(i, 0) = *reinterpret_cast<const float*>(&cloud_msg->data[offset + cloud_msg->fields[x_idx].offset]);
    points(i, 1) = *reinterpret_cast<const float*>(&cloud_msg->data[offset + cloud_msg->fields[y_idx].offset]);
    points(i, 2) = *reinterpret_cast<const float*>(&cloud_msg->data[offset + cloud_msg->fields[z_idx].offset]);
  }
  
  return points;
}

std::vector<double> GetTimestamps(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg) {
  // For now, return empty vector as timestamps are not used in this implementation
  return std::vector<double>();
}

}  // namespace utils

GroundRemovalServer::GroundRemovalServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("patchworkpp_ground_removal_node", options) {
  patchwork::Params params;
  
  // Get configuration parameters
  base_frame_ = declare_parameter<std::string>("base_frame", base_frame_);
  publish_debug_ = declare_parameter<bool>("publish_debug", publish_debug_);
  
  // Patchwork++ parameters
  params.sensor_height = declare_parameter<double>("sensor_height", params.sensor_height);
  params.num_iter      = declare_parameter<int>("num_iter", params.num_iter);
  params.num_lpr       = declare_parameter<int>("num_lpr", params.num_lpr);
  params.num_min_pts   = declare_parameter<int>("num_min_pts", params.num_min_pts);
  params.th_seeds      = declare_parameter<double>("th_seeds", params.th_seeds);

  params.th_dist    = declare_parameter<double>("th_dist", params.th_dist);
  params.th_seeds_v = declare_parameter<double>("th_seeds_v", params.th_seeds_v);
  params.th_dist_v  = declare_parameter<double>("th_dist_v", params.th_dist_v);

  params.max_range       = declare_parameter<double>("max_range", params.max_range);
  params.min_range       = declare_parameter<double>("min_range", params.min_range);
  params.uprightness_thr = declare_parameter<double>("uprightness_thr", params.uprightness_thr);

  params.verbose = get_parameter<bool>("verbose", params.verbose);

  // Support intensity via runtime parameter controlling RNR usage
  params.enable_RNR = declare_parameter<bool>("enable_RNR", params.enable_RNR);

  // Construct the main Patchwork++ node
  Patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params);

  // Initialize subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/LIDAR_TOP",
      rclcpp::SensorDataQoS(),
      std::bind(&GroundRemovalServer::removeGround, this, std::placeholders::_1));

  // QoS settings
  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  // Main output: filtered point cloud (ground removed)
  filtered_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      "/patchworkpp/filtered_cloud", qos);

  // Optional debug publishers
  if (publish_debug_) {
    ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/patchworkpp/debug/ground", qos);
    nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        "/patchworkpp/debug/nonground", qos);
  }

  RCLCPP_INFO(this->get_logger(), "Patchwork++ Ground Removal ROS 2 node initialized");
  RCLCPP_INFO(this->get_logger(), "Debug publishing: %s", publish_debug_ ? "enabled" : "disabled");
}

void GroundRemovalServer::removeGround(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  const auto &cloud = utils::PointCloud2ToEigenMat(msg);


  // Estimate ground
  Patchworkpp_->estimateGround(cloud);
  
  // Get ground and nonground points
  Eigen::MatrixX3f ground    = Patchworkpp_->getGround();
  Eigen::MatrixX3f nonground = Patchworkpp_->getNonground();
  double time_taken          = Patchworkpp_->getTimeTaken();

  // Publish the main output: filtered cloud (ground removed)
  publishFilteredCloud(nonground, msg->header);

  // Publish debug information if enabled
  if (publish_debug_) {
    publishDebugClouds(ground, nonground, msg->header);
  }

  // Log processing statistics
  RCLCPP_DEBUG(this->get_logger(), 
               "Processed %zu points: %zu ground, %zu non-ground (removed). Time: %.3f ms",
               cloud.rows(), ground.rows(), nonground.rows(), time_taken);
}

void GroundRemovalServer::publishFilteredCloud(const Eigen::MatrixX3f &nonground_points,
                                              const std_msgs::msg::Header header_msg) {
  std_msgs::msg::Header header = header_msg;
  header.frame_id = base_frame_;
  
  // Publish the filtered cloud (ground removed)
  filtered_cloud_publisher_->publish(
      std::move(utils::EigenMatToPointCloud2(nonground_points, header)));
}

void GroundRemovalServer::publishDebugClouds(const Eigen::MatrixX3f &est_ground,
                                            const Eigen::MatrixX3f &est_nonground,
                                            const std_msgs::msg::Header header_msg) {
  std_msgs::msg::Header header = header_msg;
  header.frame_id = base_frame_;
  
  // Publish ground points for debugging
  ground_publisher_->publish(
      std::move(utils::EigenMatToPointCloud2(est_ground, header)));
  
  // Publish non-ground points for debugging
  nonground_publisher_->publish(
      std::move(utils::EigenMatToPointCloud2(est_nonground, header)));
}

}  // namespace patchworkpp_ros


int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<patchworkpp_ros::GroundRemovalServer>(rclcpp::NodeOptions());
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}


