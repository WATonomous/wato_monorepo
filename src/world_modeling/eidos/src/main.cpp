#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "eidos/slam_core.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.use_intra_process_comms(false);

  auto node = std::make_shared<rclcpp::Node>("eidos_node", options);

  auto slam_core = std::make_unique<eidos::SlamCore>(node);
  slam_core->start();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);

  RCLCPP_INFO(node->get_logger(), "\033[1;32m----> Eidos SLAM Started.\033[0m");

  executor.spin();

  slam_core->stop();
  rclcpp::shutdown();
  return 0;
}
