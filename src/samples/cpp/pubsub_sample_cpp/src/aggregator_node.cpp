#include <memory>

#include "aggregator.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Aggregator>());
  rclcpp::shutdown();
  return 0;
}
