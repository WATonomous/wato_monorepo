// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ground_removal_node.hpp"

#include <functional>

#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <rclcpp/qos.hpp>

namespace wato::percpetion::patchworkpp
{

GroundRemovalNode::GroundRemovalNode(const rclcpp::NodeOptions & options)
: rclcpp::Node("patchworkpp_node", options)
{
  patchwork::Params params;
  declareParameters(params);
  core_ = std::make_unique<GroundRemovalCore>(params);

  pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    kCloudTopic,
    rclcpp::SensorDataQoS(),
    std::bind(&GroundRemovalNode::removeGround, this, std::placeholders::_1));

  rclcpp::QoS qos(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_default));
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
  qos.durability(RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL);

  ground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(kGroundTopic, qos);
  nonground_publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(kNonGroundTopic, qos);

  RCLCPP_INFO(this->get_logger(), "Patchwork++ Ground Removal ROS 2 node initialized");
  RCLCPP_INFO(
    this->get_logger(),
    "Subscribed to '%s'; publishing ground → '%s', non-ground → '%s'",
    pointcloud_sub_->get_topic_name(),
    ground_publisher_->get_topic_name(),
    nonground_publisher_->get_topic_name());
}

void GroundRemovalNode::declareParameters(patchwork::Params & params)
{
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

void GroundRemovalNode::removeGround(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  const auto & cloud = GroundRemovalCore::pointCloud2ToEigen(msg);

  core_->process(cloud);

  Eigen::MatrixX3f ground = core_->getGround();
  Eigen::MatrixX3f nonground = core_->getNonground();
  const double time_taken = core_->getTimeTaken();

  publishSegments(ground, nonground, msg->header);

  RCLCPP_DEBUG(
    this->get_logger(),
    "Processed %zu points: %zu ground, %zu non-ground (removed). Time: %.3f ms",
    cloud.rows(),
    ground.rows(),
    nonground.rows(),
    time_taken);
}

void GroundRemovalNode::publishSegments(
  const Eigen::MatrixX3f & ground_points,
  const Eigen::MatrixX3f & nonground_points,
  const std_msgs::msg::Header & in_header)
{
  if (!ground_publisher_ || !nonground_publisher_) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 5000,
      "Publishers not ready; skipping ground segmentation publish");
    return;
  }

  ground_publisher_->publish(GroundRemovalCore::eigenToPointCloud2(ground_points, in_header));
  nonground_publisher_->publish(GroundRemovalCore::eigenToPointCloud2(nonground_points, in_header));
}

}  // namespace wato::percpetion::patchworkpp

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::executors::MultiThreadedExecutor exec;
  auto node = std::make_shared<wato::percpetion::patchworkpp::GroundRemovalNode>(rclcpp::NodeOptions());
  exec.add_node(node);
  exec.spin();
  rclcpp::shutdown();
  return 0;
}
