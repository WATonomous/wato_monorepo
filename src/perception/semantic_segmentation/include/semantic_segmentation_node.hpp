#ifndef SEMANTIC_SEGMENTATION_NODE_HPP_
#define SEMANTIC_SEGMENTATION_NODE_HPP_

#include <memory>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <string>
#include <unordered_map>
#include <vector>
#include "paddle/include/paddle_inference_api.h"
#include "perception_utils/camera_utils.hpp"
#include "std_msgs/msg/string.hpp"
#include "yaml-cpp/yaml.h"

struct YamlConfig {
  std::string model_file;
  std::string params_file;
  bool is_normalize;
  bool is_resize;
  int resize_width;
  int resize_height;
};

YamlConfig load_yaml(const std::string& yaml_path);

class SemanticSegmentationNode : public rclcpp::Node {
 public:
  SemanticSegmentationNode();

 private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
  std::shared_ptr<paddle_infer::Predictor> create_predictor(const YamlConfig& yaml_config);
  void process_image(cv::Mat& img, const YamlConfig& yaml_config);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_viz_publisher_;
  int count_;
  std::shared_ptr<paddle_infer::Predictor> predictor_;
  YamlConfig yaml_config_;

  bool FLAGS_use_mkldnn;
  bool FLAGS_use_trt;
  std::string FLAGS_trt_precision;
  bool FLAGS_use_trt_dynamic_shape;
  std::string FLAGS_dynamic_shape_path;
  std::string FLAGS_devices;

  std::string image_topic;
  std::string publish_topic;
  std::string publish_vis_topic;
  std::string model_path;
  std::string save_dir;
  bool save_images;
};

#endif  // SEMANTIC_SEGMENTATION_NODE_HPP_