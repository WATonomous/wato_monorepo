#ifndef CAMERA_COMPRESSION_NODE_HPP_
#define CAMERA_COMPRESSION_NODE_HPP_

#include "rclcpp/rclcpp.hpp"

#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.hpp"
#include "sensor_msgs/msg/image.hpp"

class CameraCompressionNode : public rclcpp::Node {
  public:
    CameraCompressionNode(std::vector<std::string> image_topics, int publish_freq_ms);
  
    std::vector<image_transport::Publisher> image_publishers_;

  private:
    std::vector<std::string> image_topics_;

    rclcpp::TimerBase::SharedPtr read_carla_image_timer_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subscribers_;

    std::vector<sensor_msgs::msg::Image> carla_images_;
    
    void image_sub_callback(int i, const sensor_msgs::msg::Image::SharedPtr msg);
    void timer_callback();

};

#endif  // CAMERA_COMPRESSION_NODE_HPP_
