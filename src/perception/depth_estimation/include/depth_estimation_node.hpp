#ifndef DEPTH_ESTIMATION_NODE_HPP_
#define DEPTH_ESTIMATION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"

class DepthEstimationNode : public rclcpp::Node {
 public:
  DepthEstimationNode();

 private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  std::string camera_topic_;
  std::string depth_map_topic_;
  bool debug_node_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depth_image_publisher_;
};

#endif  // DEPTH_ESTIMATION_NODE_HPP_
