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
#include <rclcpp/parameter_value.hpp>
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

}  // namespace utils

GroundRemovalServer::GroundRemovalServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("patchworkpp_ground_removal_node", options) {
  patchwork::Params params;

  // Declare and load all parameters at once
  declareParameters(params);

  // Construct the main Patchwork++ node
  Patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params);
  // Initialize subscribers
  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GroundRemovalServer::removeGround, this, std::placeholders::_1));

  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  filtered_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_topic_, qos);

  // Optional debug publishers
  if (publish_debug_) {
    ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_topic_, qos);
    nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        nonground_topic_, qos);
  }

  RCLCPP_INFO(this->get_logger(), "Patchwork++ Ground Removal ROS 2 node initialized");
  RCLCPP_INFO(this->get_logger(), "Debug publishing: %s", publish_debug_ ? "enabled" : "disabled");
}

void GroundRemovalServer::declareParameters(patchwork::Params &params) {
  std::map<std::string, rclcpp::ParameterValue> defaults;

  // ROS node configuration defaults
  defaults["base_frame"]       = rclcpp::ParameterValue(base_frame_);
  defaults["publish_debug"]    = rclcpp::ParameterValue(publish_debug_);
  defaults["publish_original"] = rclcpp::ParameterValue(publish_original_);

  // Topics
  defaults["cloud_topic"]    = rclcpp::ParameterValue(cloud_topic_);
  defaults["filtered_topic"] = rclcpp::ParameterValue(filtered_topic_);
  defaults["ground_topic"]   = rclcpp::ParameterValue(ground_topic_);
  defaults["non_ground_topic"] = rclcpp::ParameterValue(nonground_topic_);

  // Patchwork++ algorithm parameters (seeded from library defaults)
  defaults["sensor_height"]   = rclcpp::ParameterValue(params.sensor_height);
  defaults["num_iter"]         = rclcpp::ParameterValue(params.num_iter);
  defaults["num_lpr"]          = rclcpp::ParameterValue(params.num_lpr);
  defaults["num_min_pts"]      = rclcpp::ParameterValue(params.num_min_pts);
  defaults["th_seeds"]         = rclcpp::ParameterValue(params.th_seeds);
  defaults["th_dist"]          = rclcpp::ParameterValue(params.th_dist);
  defaults["th_seeds_v"]       = rclcpp::ParameterValue(params.th_seeds_v);
  defaults["th_dist_v"]        = rclcpp::ParameterValue(params.th_dist_v);
  defaults["max_range"]        = rclcpp::ParameterValue(params.max_range);
  defaults["min_range"]        = rclcpp::ParameterValue(params.min_range);
  defaults["uprightness_thr"]  = rclcpp::ParameterValue(params.uprightness_thr);
  defaults["enable_RNR"]       = rclcpp::ParameterValue(params.enable_RNR);
  defaults["verbose"]          = rclcpp::ParameterValue(params.verbose);

  // Declare all parameters together
  this->declare_parameters("", defaults);

  // Load back into members and Patchwork++ params
  this->get_parameter("base_frame", base_frame_);
  this->get_parameter("publish_debug", publish_debug_);
  this->get_parameter("publish_original", publish_original_);

  this->get_parameter("cloud_topic", cloud_topic_);
  this->get_parameter("filtered_topic", filtered_topic_);
  this->get_parameter("ground_topic", ground_topic_);
  this->get_parameter("non_ground_topic", nonground_topic_);

  this->get_parameter("sensor_height", params.sensor_height);
  this->get_parameter("num_iter", params.num_iter);
  this->get_parameter("num_lpr", params.num_lpr);
  this->get_parameter("num_min_pts", params.num_min_pts);
  this->get_parameter("th_seeds", params.th_seeds);
  this->get_parameter("th_dist", params.th_dist);
  this->get_parameter("th_seeds_v", params.th_seeds_v);
  this->get_parameter("th_dist_v", params.th_dist_v);
  this->get_parameter("max_range", params.max_range);
  this->get_parameter("min_range", params.min_range);
  this->get_parameter("uprightness_thr", params.uprightness_thr);
  this->get_parameter("enable_RNR", params.enable_RNR);
  this->get_parameter("verbose", params.verbose);
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
  // Only publish if debug is enabled and publishers exist
  if (!publish_debug_ || !ground_publisher_ || !nonground_publisher_) {
    return;
  }
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
