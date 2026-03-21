#include <memory>

#include <rclcpp/rclcpp.hpp>

#include "eidos/slam_core.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

  auto node = std::make_shared<eidos::SlamCore>(options);

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
