#include "lanelet_service.hpp"

#include <memory>

void add(const std::shared_ptr<world_modeling_msgs::srv::LaneletService::Request> request,
          std::shared_ptr<world_modeling_msgs::srv::LaneletService::Response> response)
{
  response->sum = request->a + request->b;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\na: %ld" " b: %ld",
                request->a, request->b);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%ld]", (long int)response->sum);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);


  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("lanelet_service");

  rclcpp::Service<world_modeling_msgs::srv::LaneletService>::SharedPtr service =
    node->create_service<world_modeling_msgs::srv::LaneletService>("lanelet_service", &add);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Lanelet Service ready.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
