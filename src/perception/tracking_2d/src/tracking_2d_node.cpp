// Minimal placeholder node to unblock builds
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<rclcpp::Node>("tracking_2d_node");
  RCLCPP_INFO(node->get_logger(), "tracking_2d_node started");

  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
