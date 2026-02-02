#include <memory>

#include "tracking/tracking.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<tracking>());
  rclcpp::shutdown();
  return 0;
}