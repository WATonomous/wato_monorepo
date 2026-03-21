#include <rclcpp/rclcpp.hpp>

#include "eidos/core/eidos_node.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto node = std::make_shared<eidos::EidosNode>(options);
  executor.add_node(node->get_node_base_interface());

  RCLCPP_INFO(rclcpp::get_logger("eidos"), "Eidos node started");
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
