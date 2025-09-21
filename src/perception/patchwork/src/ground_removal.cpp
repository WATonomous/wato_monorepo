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

// Byte-swap for big-endian payloads if needed.
inline float readFloat(const uint8_t* p, bool big_endian) {
  float v;
  if (!big_endian) {
    std::memcpy(&v, p, sizeof(float));
  } else {
    uint8_t tmp[4] = {p[3], p[2], p[1], p[0]};
    std::memcpy(&v, tmp, sizeof(float));
  }
  return v;
}

inline void writeFloat(uint8_t* p, float v, bool big_endian) {
  if (!big_endian) {
    std::memcpy(p, &v, sizeof(float));
  } else {
    uint8_t tmp[4];
    std::memcpy(tmp, &v, sizeof(float));
    // write reversed
    p[0] = tmp[3]; p[1] = tmp[2]; p[2] = tmp[1]; p[3] = tmp[0];
  }
}

sensor_msgs::msg::PointCloud2 EigenMatToPointCloud2(
    const Eigen::MatrixX3f &points,
    const std_msgs::msg::Header &header) {

  sensor_msgs::msg::PointCloud2 cloud_msg;
  cloud_msg.header       = header;
  cloud_msg.height       = 1;                      
  cloud_msg.width        = static_cast<uint32_t>(points.rows());
  cloud_msg.is_bigendian = false;                  
  cloud_msg.is_dense     = false;                  

  cloud_msg.fields.clear();
  cloud_msg.fields.reserve(3);

  sensor_msgs::msg::PointField field;
  field.datatype = sensor_msgs::msg::PointField::FLOAT32;
  field.count    = 1;

  field.name   = "x"; field.offset = 0;  cloud_msg.fields.push_back(field);
  field.name   = "y"; field.offset = 4;  cloud_msg.fields.push_back(field);
  field.name   = "z"; field.offset = 8;  cloud_msg.fields.push_back(field);

  cloud_msg.point_step = 12; // 3 * 4 bytes
  cloud_msg.row_step   = cloud_msg.point_step * cloud_msg.width;

  // allocate data buffer
  cloud_msg.data.resize(static_cast<size_t>(cloud_msg.row_step) * cloud_msg.height);

  uint8_t* base = cloud_msg.data.data();
  const bool big_endian = cloud_msg.is_bigendian;

  for (int i = 0; i < points.rows(); ++i) {
    uint8_t* dst = base + static_cast<size_t>(i) * cloud_msg.point_step;
    writeFloat(dst + 0,  points(i, 0), big_endian);
    writeFloat(dst + 4,  points(i, 1), big_endian);
    writeFloat(dst + 8,  points(i, 2), big_endian);
  }

  return cloud_msg;
}

Eigen::MatrixX3f PointCloud2ToEigenMat(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg) {

  // Locate x/y/z fields and validate datatypes
  int x_idx = -1, y_idx = -1, z_idx = -1;
  for (size_t i = 0; i < cloud_msg->fields.size(); ++i) {
    const auto &f = cloud_msg->fields[i];
    if (f.name == "x") x_idx = static_cast<int>(i);
    else if (f.name == "y") y_idx = static_cast<int>(i);
    else if (f.name == "z") z_idx = static_cast<int>(i);
  }
  if (x_idx == -1 || y_idx == -1 || z_idx == -1) {
    throw std::runtime_error("PointCloud2 missing x, y, or z field");
  }
  const auto &fx = cloud_msg->fields[static_cast<size_t>(x_idx)];
  const auto &fy = cloud_msg->fields[static_cast<size_t>(y_idx)];
  const auto &fz = cloud_msg->fields[static_cast<size_t>(z_idx)];
  if (fx.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      fy.datatype != sensor_msgs::msg::PointField::FLOAT32 ||
      fz.datatype != sensor_msgs::msg::PointField::FLOAT32) {
    throw std::runtime_error("PointCloud2 x/y/z fields must be FLOAT32");
  }

  const uint32_t width      = cloud_msg->width;
  const uint32_t height     = cloud_msg->height == 0 ? 1 : cloud_msg->height; 
  const uint32_t point_step = cloud_msg->point_step;
  const uint32_t row_step   = cloud_msg->row_step ? cloud_msg->row_step : point_step * width;

  const size_t total_points = static_cast<size_t>(width) * static_cast<size_t>(height);
  Eigen::MatrixX3f points(static_cast<int>(total_points), 3);

  const uint8_t* base = cloud_msg->data.data();
  const bool big_endian = cloud_msg->is_bigendian;

  // Support both organized (height > 1) and unorganized (height == 1)
  size_t k = 0;
  for (uint32_t r = 0; r < height; ++r) {
    const uint8_t* row_ptr = base + static_cast<size_t>(r) * row_step;
    for (uint32_t c = 0; c < width; ++c, ++k) {
      const uint8_t* p = row_ptr + static_cast<size_t>(c) * point_step;

      const float x = readFloat(p + fx.offset, big_endian);
      const float y = readFloat(p + fy.offset, big_endian);
      const float z = readFloat(p + fz.offset, big_endian);

      points(static_cast<int>(k), 0) = x;
      points(static_cast<int>(k), 1) = y;
      points(static_cast<int>(k), 2) = z;
    }
  }

  return points;
}

} // namespace utils
GroundRemovalServer::GroundRemovalServer(const rclcpp::NodeOptions &options)
    : rclcpp::Node("patchworkpp_node", options) {
  patchwork::Params params;

  // load params
  declareParameters(params);

  // Patchwork++ node
  Patchworkpp_ = std::make_unique<patchwork::PatchWorkpp>(params);
  // subscribers
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
  // ROS node configuration
  publish_debug_ = this->declare_parameter<bool>("publish_debug", false);
  publish_original_ = this->declare_parameter<bool>("publish_original", false);

  // Topics
  this->declare_parameter<std::string>("base_frame", "base_link");
  cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/LIDAR_TOP");
  filtered_topic_ = this->declare_parameter<std::string>("filtered_topic", "/patchworkpp/filtered_cloud");
  ground_topic_ = this->declare_parameter<std::string>("ground_topic", "/patchworkpp/ground_cloud");
  nonground_topic_ = this->declare_parameter<std::string>("non_ground_topic", "/patchworkpp/non_ground_cloud");

  // Patchwork++ algorithm parameters (explicit defaults mirror config file)
  params.sensor_height = this->declare_parameter<double>("sensor_height", 1.88);
  params.num_iter = this->declare_parameter<int>("num_iter", 3);
  params.num_lpr = this->declare_parameter<int>("num_lpr", 20);
  params.num_min_pts = this->declare_parameter<int>("num_min_pts", 0);
  params.th_seeds = this->declare_parameter<double>("th_seeds", 0.3);
  params.th_dist = this->declare_parameter<double>("th_dist", 0.10);
  params.th_seeds_v = this->declare_parameter<double>("th_seeds_v", 0.25);
  params.th_dist_v = this->declare_parameter<double>("th_dist_v", 0.85);
  params.max_range = this->declare_parameter<double>("max_range", 80.0);
  params.min_range = this->declare_parameter<double>("min_range", 1.0);
  params.uprightness_thr = this->declare_parameter<double>("uprightness_thr", 0.101);
  params.enable_RNR = this->declare_parameter<bool>("enable_RNR", false);
  params.verbose = this->declare_parameter<bool>("verbose", true);
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
    RCLCPP_DEBUG(this->get_logger(), 
               "Processed %zu points: %zu ground, %zu non-ground (removed). Time: %.3f ms",
               cloud.rows(), ground.rows(), nonground.rows(), time_taken);
  }
}
  
void GroundRemovalServer::publishFilteredCloud(
    const Eigen::MatrixX3f &nonground_points,
    const std_msgs::msg::Header &in_header) {

  filtered_cloud_publisher_->publish(
      utils::EigenMatToPointCloud2(nonground_points, in_header));
}

void GroundRemovalServer::publishDebugClouds(
    const Eigen::MatrixX3f &est_ground,
    const Eigen::MatrixX3f &est_nonground,
    const std_msgs::msg::Header &in_header) {

  if (!publish_debug_ || !ground_publisher_ || !nonground_publisher_) return;

  ground_publisher_->publish(utils::EigenMatToPointCloud2(est_ground, in_header));
  nonground_publisher_->publish(utils::EigenMatToPointCloud2(est_nonground, in_header));
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
