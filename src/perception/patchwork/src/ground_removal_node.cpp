#include "ground_removal_node.hpp"

#include <functional>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/qos.hpp>

namespace patchworkpp_ros {

GroundRemovalNode::GroundRemovalNode(const rclcpp::NodeOptions &options)
    : rclcpp::Node("patchworkpp_node", options) {
  patchwork::Params params;
  declareParameters(params);
  core_ = std::make_unique<GroundRemovalCore>(params);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_,
      rclcpp::SensorDataQoS(),
      std::bind(&GroundRemovalNode::removeGround, this, std::placeholders::_1));

  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  filtered_cloud_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
      filtered_topic_, qos);

  if (publish_debug_) {
    ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        ground_topic_, qos);
    nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        nonground_topic_, qos);
  }

  RCLCPP_INFO(this->get_logger(), "Patchwork++ Ground Removal ROS 2 node initialized");
  RCLCPP_INFO(this->get_logger(), "Debug publishing: %s", publish_debug_ ? "enabled" : "disabled");
}

void GroundRemovalNode::declareParameters(patchwork::Params &params) {
  publish_debug_ = this->declare_parameter<bool>("publish_debug", false);
  publish_original_ = this->declare_parameter<bool>("publish_original", false);

  cloud_topic_ = this->declare_parameter<std::string>("cloud_topic", "/LIDAR_TOP");
  filtered_topic_ = this->declare_parameter<std::string>("filtered_topic", "/patchworkpp/filtered_cloud");
  ground_topic_ = this->declare_parameter<std::string>("ground_topic", "/patchworkpp/ground_cloud");
  nonground_topic_ = this->declare_parameter<std::string>("non_ground_topic", "/patchworkpp/non_ground_cloud");

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

void GroundRemovalNode::removeGround(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
  const auto &cloud = GroundRemovalCore::pointCloud2ToEigen(msg);

  core_->process(cloud);

  Eigen::MatrixX3f ground = core_->getGround();
  Eigen::MatrixX3f nonground = core_->getNonground();
  const double time_taken = core_->getTimeTaken();

  publishFilteredCloud(nonground, msg->header);

  if (publish_debug_) {
    publishDebugClouds(ground, nonground, msg->header);
    RCLCPP_DEBUG(this->get_logger(),
                 "Processed %zu points: %zu ground, %zu non-ground (removed). Time: %.3f ms",
                 cloud.rows(), ground.rows(), nonground.rows(), time_taken);
  }
}

void GroundRemovalNode::publishFilteredCloud(
    const Eigen::MatrixX3f &nonground_points,
    const std_msgs::msg::Header &in_header) {
  filtered_cloud_publisher_->publish(
      GroundRemovalCore::eigenToPointCloud2(nonground_points, in_header));
}

void GroundRemovalNode::publishDebugClouds(
    const Eigen::MatrixX3f &est_ground,
    const Eigen::MatrixX3f &est_nonground,
    const std_msgs::msg::Header &in_header) {
  if (!publish_debug_ || !ground_publisher_ || !nonground_publisher_) {
    return;
  }

  ground_publisher_->publish(GroundRemovalCore::eigenToPointCloud2(est_ground, in_header));
  nonground_publisher_->publish(GroundRemovalCore::eigenToPointCloud2(est_nonground, in_header));
}

}  // namespace patchworkpp_ros

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<patchworkpp_ros::GroundRemovalNode>(rclcpp::NodeOptions());
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
