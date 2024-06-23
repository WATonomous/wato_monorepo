#include <memory>
#include <iostream>
#include "occupancy_segmentation_node.hpp"

OccupancySegmentationNode::OccupancySegmentationNode() : Node("occupancy_segmentation") {

  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&OccupancySegmentationNode::timer_callback, this));
}

void OccupancySegmentationNode::timer_callback(){
  RCLCPP_INFO(this->get_logger(), "Working");
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancySegmentationNode>());
  rclcpp::shutdown();
  return 0;
}
