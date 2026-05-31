#ifndef CAMERA_DEBAYER__CAMERA_DEBAYER_HPP_
#define CAMERA_DEBAYER__CAMERA_DEBAYER_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "isaac_ros_managed_nitros/managed_nitros_publisher.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "isaac_ros_nitros_image_type/nitros_image.hpp"
#include "camera_debayer/camera_debayer_core.hpp"

#include "cuda_runtime.h"  // NOLINT - include .h without directory

namespace wato::perception::camera_debayer
{

class CameraDebayerNode : public rclcpp::Node
{
public:
  explicit CameraDebayerNode(const rclcpp::NodeOptions options = rclcpp::NodeOptions());

  ~CameraDebayerNode();

private:
  void InputCallback(const sensor_msgs::msg::Image::SharedPtr msg);

  // Subscription to input Image messages
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;

  // Publisher for output NitrosImage messages
  std::shared_ptr<nvidia::isaac_ros::nitros::ManagedNitrosPublisher<
      nvidia::isaac_ros::nitros::NitrosImage>> nitros_pub_;

  // CUDA stream to process dynamics detection on
  cudaStream_t cuda_stream_;
  
  CameraDebayerCore core_;
};

}  // namespace wato::perception::camera_debayer

#endif  // CAMERA_DEBAYER__CAMERA_DEBAYER_HPP_
