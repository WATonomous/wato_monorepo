#include <memory>

#include "localization_node.hpp"

LocalizationNode::LocalizationNode() : Node("sample"), sample_(world_modeling::hd_map::Sample())
{
  gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      "/gps", ADVERTISING_FREQ,
      std::bind(&LocalizationNode::sample_gps_callback, this, std::placeholders::_1));

  pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      "/pose", ADVERTISING_FREQ,
      std::bind(&LocalizationNode::sample_pose_callback, this, std::placeholders::_1));

  sample_pub_ =
      this->create_publisher<sample_msgs::msg::Unfiltered>("hd_map_sample_topic", ADVERTISING_FREQ);

  // Define the default values for parameters if not defined in params.yaml
  this->declare_parameter("version", rclcpp::ParameterValue(0));
  this->declare_parameter("compression_method", rclcpp::ParameterValue(0));
}

void LocalizationNode::sample_gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Recieved GPS data.. \n");
}

void LocalizationNode::sample_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Recieved POSE data.. \n");
  RCLCPP_INFO(this->get_logger(), "x: %f, y: %f, z: %f\n", msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
}

void LocalizationNode::sample_publish(const sample_msgs::msg::Unfiltered::SharedPtr msg)
{
  auto pub_msg = sample_msgs::msg::Unfiltered();
  RCLCPP_INFO(this->get_logger(), "Publishing Sample Message from Localization...\n");
  sample_pub_->publish(pub_msg);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalizationNode>());
  rclcpp::shutdown();
  return 0;
}