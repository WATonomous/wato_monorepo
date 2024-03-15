#include <chrono>
#include <filesystem>
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
    this->declare_parameter<std::string>("camera_topic", "/CAM_FRONT/image_rect_compressed");
    this->declare_parameter<std::string>("publish_vis_topic", "/CAM_FRONT/lanes_viz");
    this->declare_parameter<std::string>("publish_lanes_topic", "/CAM_FRONT/lanes");
    this->declare_parameter<bool>("save_images", false);
    this->declare_parameter<std::string>("save_dir", "/tmp");
    this->declare_parameter<bool>("publish_source_image", false);

    this->get_parameter("camera_topic", camera_topic_);
    this->get_parameter("publish_vis_topic", publish_vis_topic_);
    this->get_parameter("publish_lanes_topic", publish_lanes_topic_);
    this->get_parameter("save_images", save_images_);
    this->get_parameter("save_dir", save_dir_);
    this->get_parameter("publish_source_image", publish_source_image_);

    if (!std::filesystem::exists(save_dir_)) {
      std::filesystem::create_directories(save_dir_);
    }

    RCLCPP_INFO(this->get_logger(), "Subscribing to camera topic: %s", camera_topic_.c_str());

    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topic_, 10,
        std::bind(&LaneDetectionNode::image_callback, this, std::placeholders::_1));

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(publish_vis_topic_, 10);
    lane_detection_pub_ =
        this->create_publisher<lane_detection_msgs::msg::LaneDetection>(publish_lanes_topic_, 10);
  }

  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat image = cv_ptr->image;

    if (image.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Decoded image is empty!");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Got Decoded image: width=%d, height=%d, type=%d", image.cols,
                image.rows, image.type());

    RCLCPP_INFO(this->get_logger(), "=== Start frame ===");
    const auto &time_all0 = std::chrono::steady_clock::now();

    std::vector<std::vector<float>> raw_lane_list;

    /* Call image processor library */
    ImageProcessor::Result result;
    ImageProcessor::Process(image, result, raw_lane_list);
    const auto &time_all1 = std::chrono::steady_clock::now();
    double time_all = (time_all1 - time_all0).count() / 1000000.0;
    RCLCPP_INFO(this->get_logger(), "Total:               %9.3lf [msec]", time_all);
    RCLCPP_INFO(this->get_logger(), "    Pre processing:  %9.3lf [msec]", result.time_pre_process);
    RCLCPP_INFO(this->get_logger(), "    Inference:       %9.3lf [msec]", result.time_inference);
    RCLCPP_INFO(this->get_logger(), "    Post processing: %9.3lf [msec]", result.time_post_process);
    RCLCPP_INFO(this->get_logger(), "=== Finished frame ===");

    // Convert the processed cv::Mat back to sensor_msgs::msg::Image and publish
    sensor_msgs::msg::Image::SharedPtr img_msg =
        cv_bridge::CvImage(msg->header, "bgr8", image).toImageMsg();
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

    if (publish_source_image_) {
      lane_msg.source_img = *img_msg;
    }

    if (save_images_) {
      std::string filename = save_dir_ + "/lane_detection_" + std::to_string(count) + ".png";
      cv::imwrite(filename, image);
      RCLCPP_INFO(this->get_logger(), "Saved image to %s", filename.c_str());
    }

    lane_detection_pub_->publish(lane_msg);
    count++;
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;  // The image publisher
  rclcpp::Publisher<lane_detection_msgs::msg::LaneDetection>::SharedPtr
      lane_detection_pub_;  // The lane detection publisher

  size_t count_;

  // ROS Parameters
  std::string camera_topic_;
  std::string publish_vis_topic_;
  std::string publish_lanes_topic_;
  bool save_images_;
  std::string save_dir_;
  bool publish_source_image_;
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