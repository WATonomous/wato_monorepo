#include <memory>

#include "transformer.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Transformer>());
  rclcpp::shutdown();
  return 0;
}
