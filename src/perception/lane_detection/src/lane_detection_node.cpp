#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"

#include "lane_detection_msgs/msg/lane_detection.hpp"
#include "lane_engine.h"
#include "std_msgs/msg/string.hpp"

#include <opencv2/opencv.hpp>

#include "common_helper_cv.h"
#include "image_processor.h"

using namespace std::chrono_literals;

/*** Macro ***/
#define WORK_DIR RESOURCE_DIR

int count = 0;

class LaneDetectionNode : public rclcpp::Node {
 public:
  LaneDetectionNode() : Node("lane_detection"), count_(0) {
    std::string input_topic;
    this->declare_parameter<std::string>("input_topic", "/CAM_FRONT/image_rect_compressed");
    this->get_parameter("input_topic", input_topic);
    RCLCPP_INFO(this->get_logger(), "Subscribing to camera topic: %s", input_topic.c_str());

    subscription_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        input_topic, 10,
        std::bind(&LaneDetectionNode::image_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>("lane_detection_image", 10);
    lane_detection_pub_ =
        this->create_publisher<lane_detection_msgs::msg::LaneDetection>("lane_detection", 10);
  }

  void image_callback(const sensor_msgs::msg::CompressedImage::ConstSharedPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image.clone();

    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Decoded image is empty!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Got Decoded image: width=%d, height=%d, type=%d", image.cols,
                image.rows, image.type());

    RCLCPP_INFO(this->get_logger(), "=== Start frame ===");
    const auto &time_all0 = std::chrono::steady_clock::now();
    /* Read image */
    const auto &time_cap0 = std::chrono::steady_clock::now();

    const auto &time_cap1 = std::chrono::steady_clock::now();

    std::vector<std::vector<float>> raw_lane_list;

    /* Call image processor library */
    const auto &time_image_process0 = std::chrono::steady_clock::now();
    ImageProcessor::Result result;
    ImageProcessor::Process(image, result, raw_lane_list);
    const auto &time_image_process1 = std::chrono::steady_clock::now();

    /* Print processing time */
    const auto &time_all1 = std::chrono::steady_clock::now();
    double time_all = (time_all1 - time_all0).count() / 1000000.0;
    double time_cap = (time_cap1 - time_cap0).count() / 1000000.0;
    double time_image_process = (time_image_process1 - time_image_process0).count() / 1000000.0;
    RCLCPP_INFO(this->get_logger(), "Total:               %9.3lf [msec]", time_all);
    RCLCPP_INFO(this->get_logger(), "  Capture:           %9.3lf [msec]", time_cap);
    RCLCPP_INFO(this->get_logger(), "  Image processing:  %9.3lf [msec]", time_image_process);
    RCLCPP_INFO(this->get_logger(), "    Pre processing:  %9.3lf [msec]", result.time_pre_process);
    RCLCPP_INFO(this->get_logger(), "    Inference:       %9.3lf [msec]", result.time_inference);
    RCLCPP_INFO(this->get_logger(), "    Post processing: %9.3lf [msec]", result.time_post_process);
    RCLCPP_INFO(this->get_logger(), "=== Finished frame ===");

    // Convert the processed cv::Mat back to sensor_msgs::msg::Image and publish
    sensor_msgs::msg::Image::SharedPtr img_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image).toImageMsg();
    image_pub_->publish(*img_msg);

    // Create the lane detection message
    lane_detection_msgs::msg::LaneDetection lane_msg;
    lane_msg.header.stamp = this->now();
    lane_msg.header.frame_id = "lane_detection";
    lane_msg.lines.clear();

    // Load the message from the raw lane list
    for (size_t i = 0; i < raw_lane_list.size(); i++) {
      lane_detection_msgs::msg::LaneLine line;
      line.points.resize(raw_lane_list[i].size());
      for (size_t j = 0; j < raw_lane_list[i].size() / 2; j++) {
        line.points[j].x = raw_lane_list[i][j * 2 + 0];
        line.points[j].y = raw_lane_list[i][j * 2 + 1];
        line.points[j].z = 0;
      }
      lane_msg.lines.push_back(line);
    }

    lane_detection_pub_->publish(lane_msg);
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;  // The image publisher
  rclcpp::Publisher<lane_detection_msgs::msg::LaneDetection>::SharedPtr
      lane_detection_pub_;  // The lane detection publisher

  size_t count_;
};

int main(int argc, char *argv[]) {
  /* Initialize image processor library */
  ImageProcessor::InputParam input_param = {WORK_DIR, 4};
  if (ImageProcessor::Initialize(input_param) != 0) {
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