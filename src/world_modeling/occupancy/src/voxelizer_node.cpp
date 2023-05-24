#include <memory>

#include "voxelizer_node.hpp"

VoxelizerNode::VoxelizerNode() : Node("voxelizer") {
  RCLCPP_INFO(this->get_logger(), "Voxelizer Node\n");
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VoxelizerNode>());
  rclcpp::shutdown();
  return 0;
}
