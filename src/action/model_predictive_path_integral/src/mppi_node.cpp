#include "rclcpp/rclcpp.hpp"
#include <torch/torch.h>
#include <iostream>

class MppiNode : public rclcpp::Node {
public:
  MppiNode() : Node("mppi_node") {
    RCLCPP_INFO(this->get_logger(), "MPPI node starting…");

    // Simple libtorch test: tensor creation + addition
    torch::Tensor a = torch::rand({2, 3});
    torch::Tensor b = torch::rand({2, 3});
    torch::Tensor c = a + b;

    std::cout << "Tensor a:\n" << a << std::endl;
    std::cout << "Tensor b:\n" << b << std::endl;
    std::cout << "Tensor c = a + b:\n" << c << std::endl;
    
    RCLCPP_INFO(this->get_logger(), "Libtorch test complete.");
  }
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MppiNode>());
  rclcpp::shutdown();
  return 0;
}
