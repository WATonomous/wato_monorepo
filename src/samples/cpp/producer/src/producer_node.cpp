#include <memory>

#include "producer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Producer>());
  rclcpp::shutdown();
  return 0;
}
