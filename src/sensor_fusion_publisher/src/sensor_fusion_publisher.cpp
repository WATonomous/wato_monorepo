#include <cstdio>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "common_msgs/msg/combined_sensor.hpp"


class SensorFusionNode : public rclcpp::Node
{
public:
  SensorFusionNode() : Node("sensor_fusion_node")
  {
    combined_publisher_ = this->create_publisher<common_msgs::msg::CombinedSensor>("combined_topic", 10);

    camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/camera_topic", 10, std::bind(&SensorFusionNode::camera_callback, this, std::placeholders::_1));

    lidar_subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/lidar_topic", 10, std::bind(&SensorFusionNode::lidar_callback, this, std::placeholders::_1));
  }

private:
  void camera_callback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    last_camera_msg_ = msg;
    publish_combined_msg();
  }

  void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    last_lidar_msg_ = msg;
    publish_combined_msg();
  }

  void publish_combined_msg()
  {
    if (last_camera_msg_ != nullptr && last_lidar_msg_ != nullptr) {
      auto combined_msg = common_msgs::msg::CombinedSensor();
      combined_msg.image = *last_camera_msg_;
      combined_msg.lidar = *last_lidar_msg_;
      combined_publisher_->publish(combined_msg);
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_subscription_;
  rclcpp::Publisher<common_msgs::msg::CombinedSensor>::SharedPtr combined_publisher_;

  sensor_msgs::msg::Image::SharedPtr last_camera_msg_ = nullptr;
  sensor_msgs::msg::PointCloud2::SharedPtr last_lidar_msg_ = nullptr;
};

int main(int argc, char * argv[])
{
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting sensor_fusion_publisher...");
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorFusionNode>());
  rclcpp::shutdown();
  return 0;
}