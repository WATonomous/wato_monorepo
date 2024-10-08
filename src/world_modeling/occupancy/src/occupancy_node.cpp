#include "occupancy_node.hpp"

#include <memory>
#include <string>


Occupancy::OccupancyNode() : Node("occupancy") {
  // Declare ROS parameters
  this->declare_parameter<std::string>("subscription_topic", std::string("/nonground_points"));
  this->declare_parameter<std::string>("publish_topic", std::string("/2d_points"));
  
  // Fetch parameters
  auto input_topic = this->get_parameter("subscription_topic").as_string();
  auto output_topic = this->get_parameter("publish_topic").as_string();
  
  _subscriber = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      intput_topic, 10,
      std::bind(&OccupancyNode::subscription_callback, this, std::placeholders::_1));

  _publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(output_topic, 10);
}

void OccupancynNode::subscription_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
  RCLCPP_INFO(this -> get_logger(), "Received message: %s", msg->header.frame_id.c_str());
  // pcl::fromROSMsg(*lidar_cloud, temp_cloud);

  output_cloud.header = msg->header;

  auto start = Clock::now();
  sensor_msgs::msg::PointCloud2 output_cloud = remove_z_dimension(msg);
  auto end = Clock::now();

  int duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
  RCLCPP_INFO(this->get_logger(), "Runtime for dimension reduction: %i ms", duration);
  RCLCPP_INFO(this->get_logger(), "3D points: %i", static_cast<int>(msg->width));
  RCLCPP_INFO(this->get_logger(), "2D points: %i", static_cast<int>(output_cloud.width));

  // Check if the input and output widths match
  if (msg->width != output_cloud.width) {
      RCLCPP_WARN(this->get_logger(), "Mismatch in point counts: input width %i, output width %i", static_cast<int>(msg->width), static_cast<int>(output_cloud.width));
  }
  RCLCPP_INFO(this->get_logger(), "Header outgoing: %s", output_cloud.header.frame_id.c_str());

  _publisher->publish(output_cloud);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OccupancyNode>());
  rclcpp::shutdown();
  return 0;
}
