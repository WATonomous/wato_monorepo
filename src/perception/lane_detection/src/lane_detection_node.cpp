#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <cv_bridge/cv_bridge.h>
#include "image_transport/image_transport.hpp"

#include "std_msgs/msg/string.hpp"


#include <opencv2/opencv.hpp>

#include "common_helper_cv.h"
#include "image_processor.h"


using namespace std::chrono_literals;

/*** Macro ***/
#define WORK_DIR RESOURCE_DIR
#define DEFAULT_INPUT_IMAGE RESOURCE_DIR "/dashcam_01.jpg"
#define LOOP_NUM_FOR_TIME_MEASUREMENT 10

int count = 0;

class LaneDetectionNode : public rclcpp::Node
{
public:
  LaneDetectionNode()
      : Node("lane_detection"), count_(0)
  {

    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "/CAM_FRONT/image_rect_compressed", 10,
        std::bind(&LaneDetectionNode::image_callback, this, std::placeholders::_1));

    // Initialize the image publisher using rclcpp
    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("lane_detection_image", 10);
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg)
  {

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image.clone();

    if (image.empty())
    {
      RCLCPP_ERROR(this->get_logger(), "Decoded image is empty!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Got Decoded image: width=%d, height=%d, type=%d", image.cols, image.rows, image.type());

    printf("=== Start frame ===\n");
    const auto &time_all0 = std::chrono::steady_clock::now();
    /* Read image */
    const auto &time_cap0 = std::chrono::steady_clock::now();

    const auto &time_cap1 = std::chrono::steady_clock::now();

    /* Call image processor library */
    const auto &time_image_process0 = std::chrono::steady_clock::now();
    ImageProcessor::Result result;
    ImageProcessor::Process(image, result);
    const auto &time_image_process1 = std::chrono::steady_clock::now();

    /* Print processing time */
    const auto &time_all1 = std::chrono::steady_clock::now();
    double time_all = (time_all1 - time_all0).count() / 1000000.0;
    double time_cap = (time_cap1 - time_cap0).count() / 1000000.0;
    double time_image_process = (time_image_process1 - time_image_process0).count() / 1000000.0;
    printf("Total:               %9.3lf [msec]\n", time_all);
    printf("  Capture:           %9.3lf [msec]\n", time_cap);
    printf("  Image processing:  %9.3lf [msec]\n", time_image_process);
    printf("    Pre processing:  %9.3lf [msec]\n", result.time_pre_process);
    printf("    Inference:       %9.3lf [msec]\n", result.time_inference);
    printf("    Post processing: %9.3lf [msec]\n", result.time_post_process);
    printf("=== Finished frame ===\n\n");

    // Convert the processed cv::Mat back to sensor_msgs::msg::Image and publish
    sensor_msgs::msg::Image::SharedPtr img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    image_pub_->publish(*img_msg);
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_; // The image publisher

  size_t count_;
};

int main(int argc, char *argv[])
{
  /* Initialize image processor library */
  ImageProcessor::InputParam input_param = {WORK_DIR, 4};
  if (ImageProcessor::Initialize(input_param) != 0)
  {
    printf("Initialization Error\n");
    return -1;
  }
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Initializing node");
  rclcpp::init(argc, argv);

  std::shared_ptr<LaneDetectionNode> node = std::make_shared<LaneDetectionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}