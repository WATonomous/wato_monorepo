#include <chrono>
#include <memory>
#include <vector>

#include "occupancy_segmentation_node.hpp"
// #include "std_msgs/msg/int32.hpp"

OccupancySegmentationNode::OccupancySegmentationNode(int delay_ms)
: Node("occupancy_segmentation")
{
  data_pub_ =
    this->create_publisher<std_msgs::msg::Int32>("/jack_test_topic", ADVERTISING_FREQ);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(delay_ms),
    std::bind(&OccupancySegmentationNode::timer_callback, this));
}

void OccupancySegmentationNode::timer_callback()
{
  // occupancy_segmentation_.update_coordinates();
  auto msg = std_msgs::msg::Int32();
  msg.data = 1;
  // msg.timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
  //   std::chrono::system_clock::now().time_since_epoch()).count();
  // occupancy_segmentation_.serialize_coordinates(msg);

  RCLCPP_INFO(this->get_logger(), "Publishing: %llu", msg.data);
  data_pub_->publish(msg);
}


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancySegmentationNode>(500));
  rclcpp::shutdown();
  return 0;
}
