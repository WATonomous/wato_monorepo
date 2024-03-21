#include "depth_estimation_node.hpp"
#include "sensor_msgs/msg/image.hpp"

DepthEstimationNode::DepthEstimationNode() : Node("depth_estimation_node") {
  this->declare_parameter<std::string>("camera_topic", "/camera/image_raw");
  this->declare_parameter<std::string>("depth_map_topic", "/depth/image_raw");
  this->declare_parameter<bool>("deubg_node", false);

  this->get_parameter("camera_topic", camera_topic_);
  this->get_parameter("depth_map_topic", depth_map_topic_);
  this->get_parameter("debug_node", debug_node_);

  RCLCPP_INFO(this->get_logger(), "debug_node: %s", debug_node_ ? "true" : "false");

  RCLCPP_INFO(this->get_logger(), "Subscribing to camera topic '%s'", camera_topic_.c_str());
  image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      camera_topic_, 10,
      std::bind(&DepthEstimationNode::image_callback, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Publishing depth map to topic '%s'", depth_map_topic_.c_str());
  depth_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(depth_map_topic_, 10);
}

void DepthEstimationNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
  RCLCPP_INFO(this->get_logger(), "Received sensor image on '%s'", camera_topic_.c_str());

  // TODO: Process the incoming image and estimate depth
  // Replace this empty image with the actual depth map
  auto depth_map_image = sensor_msgs::msg::Image();
  depth_image_publisher_->publish(depth_map_image);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DepthEstimationNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
