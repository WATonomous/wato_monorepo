#include "camera_debayer/camera_debayer_node.hpp"
#include <cuda_runtime.h>
#include "isaac_ros_nitros_image_type/nitros_image_builder.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "isaac_ros_common/cuda_stream.hpp"

namespace wato::perception::camera_debayer
{

CameraDebayerNode::CameraDebayerNode(const rclcpp::NodeOptions options)
: rclcpp::Node("camera_debayer_node", options),
  // Subscribes to image_raw
  sub_{create_subscription<sensor_msgs::msg::Image>(
      "image_raw", 10,
      std::bind(&CameraDebayerNode::InputCallback, this, std::placeholders::_1))},
  // publishes to image_color
  nitros_pub_{std::make_shared<nvidia::isaac_ros::nitros::ManagedNitrosPublisher
        nvidia::isaac_ros::nitros::NitrosImage>>(
      this, "image_color",
      nvidia::isaac_ros::nitros::nitros_image_bgr8_t::supported_type_name)}
{
  CHECK_CUDA_ERROR(
    ::nvidia::isaac_ros::common::initNamedCudaStream(
      cuda_stream_, "isaac_ros_gpu_image_builder_node"),
    "Error initializing CUDA stream");
}

CameraDebayerNode::~CameraDebayerNode() = default;

void CameraDebayerNode::InputCallback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  // Get size of image in bytes
  size_t buffer_size{msg->step * msg->height};

  // Allocate CUDA buffer (GPU memory) to store image
  void * buffer;
  CHECK_CUDA_ERROR(
    cudaMallocAsync(&buffer, buffer_size, cuda_stream_),
    "Error allocating CUDA buffer");

  // Copy data bytes to CUDA buffer
  CHECK_CUDA_ERROR(
    cudaMemcpyAsync(
      buffer, msg->data.data(), buffer_size, cudaMemcpyDefault,
      cuda_stream_),
    "Error copying data to CUDA buffer");

  // Call core to debayer on GPU (Bayer → BGR)
  void * bgr_buffer = core_.debayer(
    buffer, msg->width, msg->height, msg->step, cuda_stream_);

  // Copies header from imcoming message (timestamp and frame_id)
  std_msgs::msg::Header header;
  header = msg->header;

  // Waits for all GPU operations to finish before moving on
  CHECK_CUDA_ERROR(cudaStreamSynchronize(cuda_stream_), "Error synchronizing CUDA stream");

  // Create NitrosImage wrapping CUDA buffer
  nvidia::isaac_ros::nitros::NitrosImage nitros_image =
    nvidia::isaac_ros::nitros::NitrosImageBuilder()
    .WithHeader(header)
    .WithEncoding(sensor_msgs::image_encodings::BGR8)
    .WithDimensions(msg->height, msg->width)
    .WithGpuData(bgr_buffer)
    .Build();

  nitros_pub_->publish(nitros_image);
  RCLCPP_INFO(this->get_logger(), "Sent CUDA buffer with memory at: %p", buffer);
}

}  // namespace wato::perception::camera_debayer

// Register as component
#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(wato::perception::camera_debayer::CameraDebayerNode)