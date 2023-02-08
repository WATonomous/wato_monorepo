#include "continental_node.hpp"

ContinentalNode::ContinentalNode()
: Node("continental_driver"),
  driver_(continental_driver::ContinentalDriver())
{}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContinentalNode>());
    rclcpp::shutdown();
    return 0;
}
