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
} YamlConfig;

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

  bool FLAGS_use_mkldnn = true;
  bool FLAGS_use_trt = true;
  std::string FLAGS_trt_precision = "fp32";
  bool FLAGS_use_trt_dynamic_shape = true;
  std::string FLAGS_dynamic_shape_path = resource_path + "dynamic_shape.pbtxt";
  std::string FLAGS_devices = "GPU";

  std::shared_ptr<paddle_infer::Predictor> create_predictor(const YamlConfig& yaml_config) {
    paddle_infer::Config infer_config;
    infer_config.SetModel(model_path + "/" + yaml_config.model_file,
                          model_path + "/" + yaml_config.params_file);
    infer_config.EnableMemoryOptim();

    if (FLAGS_devices == "CPU") {
      RCLCPP_INFO(this->get_logger(), "Use CPU");
      if (FLAGS_use_mkldnn) {
        RCLCPP_INFO(this->get_logger(), "Use MKLDNN");
        infer_config.EnableMKLDNN();
        infer_config.SetCpuMathLibraryNumThreads(5);
      }
    } else if (FLAGS_devices == "GPU") {
      RCLCPP_INFO(this->get_logger(), "Use GPU");
      infer_config.EnableUseGpu(100, 0);

      // TRT config
      if (FLAGS_use_trt) {
        //   LOG(INFO) << "Use TRT";
        //   LOG(INFO) << "trt_precision:" << FLAGS_trt_precision;
        RCLCPP_INFO(this->get_logger(), "trt_precision: %s", FLAGS_trt_precision.c_str());

        // TRT precision
        if (FLAGS_trt_precision == "fp32") {
          infer_config.EnableTensorRtEngine(8ULL << 30, 1, 3, paddle_infer::PrecisionType::kFloat32,
                                            false, false);
        } else if (FLAGS_trt_precision == "fp16") {
          infer_config.EnableTensorRtEngine(8ULL << 30, 1, 3, paddle_infer::PrecisionType::kHalf,
                                            false, false);
        } else if (FLAGS_trt_precision == "int8") {
          infer_config.EnableTensorRtEngine(8ULL << 30, 1, 3, paddle_infer::PrecisionType::kInt8,
                                            false, false);
        } else {
          RCLCPP_ERROR(this->get_logger(), "The trt_precision should be fp32, fp16 or int8.");
        }

        // TRT dynamic shape
        if (FLAGS_use_trt_dynamic_shape) {
          RCLCPP_INFO(this->get_logger(), "Enable TRT dynamic shape");
          RCLCPP_INFO(this->get_logger(), "dynamic_shape_path: %s",
                      FLAGS_dynamic_shape_path.c_str());
          if (FLAGS_dynamic_shape_path.empty()) {
            std::map<std::string, std::vector<int>> min_input_shape = {{"image", {1, 3, 112, 112}}};
            std::map<std::string, std::vector<int>> max_input_shape = {
                {"image", {1, 3, 1024, 2048}}};
            std::map<std::string, std::vector<int>> opt_input_shape = {
                {"image", {1, 3, 512, 1024}}};
            infer_config.SetTRTDynamicShapeInfo(min_input_shape, max_input_shape, opt_input_shape);
          } else {
            infer_config.EnableTunedTensorRtDynamicShape(FLAGS_dynamic_shape_path, true);
          }
        }
      }
    } else {
      RCLCPP_ERROR(this->get_logger(), "The devices should be GPU or CPU");
    }

    auto predictor = paddle_infer::CreatePredictor(infer_config);
    return predictor;
  }

  SemanticSegmentationNode() : Node("semantic_segmentation_node"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ =
        this->create_wall_timer(500ms, std::bind(&SemanticSegmentationNode::timer_callback, this));
    YamlConfig yaml_config = load_yaml(resource_path + "deploy.yaml");
    // Print out the yaml config
    RCLCPP_INFO(this->get_logger(), "model_file: %s", yaml_config.model_file.c_str());
    RCLCPP_INFO(this->get_logger(), "params_file: %s", yaml_config.params_file.c_str());
    RCLCPP_INFO(this->get_logger(), "is_normalize: %d", yaml_config.is_normalize);
    RCLCPP_INFO(this->get_logger(), "is_resize: %d", yaml_config.is_resize);

    // Create the paddle predictor
    predictor_ = create_predictor(yaml_config);
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
  std::shared_ptr<paddle_infer::Predictor> predictor_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}