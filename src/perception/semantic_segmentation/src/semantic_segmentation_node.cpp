#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "paddle/include/paddle_inference_api.h"
#include "yaml-cpp/yaml.h"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

std::string model_path = "/perception_models/semantic_segmentation/pp_liteseg_infer_model/";
std::string resource_path = "/home/bolty/ament_ws/src/semantic_segmentation/resource/";



typedef struct YamlConfig {
  std::string model_file;
  std::string params_file;
  bool is_normalize;
  bool is_resize;
  int resize_width;
  int resize_height;
}YamlConfig;

YamlConfig load_yaml(const std::string& yaml_path) {
  YAML::Node node = YAML::LoadFile(yaml_path);
  std::string model_file = node["Deploy"]["model"].as<std::string>();
  std::string params_file = node["Deploy"]["params"].as<std::string>();
  YamlConfig yaml_config = {model_file, params_file};
  if (node["Deploy"]["transforms"]) {
    const YAML::Node& transforms = node["Deploy"]["transforms"];
    for (size_t i = 0; i < transforms.size(); i++) {
      if (transforms[i]["type"].as<std::string>() == "Normalize") {
        yaml_config.is_normalize = true;
      } else if (transforms[i]["type"].as<std::string>() == "Resize") {
        yaml_config.is_resize = true;
        const YAML::Node& target_size = transforms[i]["target_size"];
        yaml_config.resize_width = target_size[0].as<int>();
        yaml_config.resize_height = target_size[1].as<int>();
      }
    }
  }
  return yaml_config;
}

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class SemanticSegmentationNode : public rclcpp::Node {
 public:
  SemanticSegmentationNode() : Node("semantic_segmentation_node"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(500ms, std::bind(&SemanticSegmentationNode::timer_callback, this));
    YamlConfig yaml_config = load_yaml(resource_path + "deploy.yaml");
    // Print out the yaml config
    RCLCPP_INFO(this->get_logger(), "model_file: %s", yaml_config.model_file.c_str());
    RCLCPP_INFO(this->get_logger(), "params_file: %s", yaml_config.params_file.c_str());
    RCLCPP_INFO(this->get_logger(), "is_normalize: %d", yaml_config.is_normalize);
    RCLCPP_INFO(this->get_logger(), "is_resize: %d", yaml_config.is_resize);
  }

 private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world!! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}