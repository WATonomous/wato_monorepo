#include <cv_bridge/cv_bridge.h>
#include <chrono>
#include <functional>
#include <memory>
#include <opencv2/opencv.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "paddle/include/paddle_inference_api.h"
#include "yaml-cpp/yaml.h"

#include <sensor_msgs/msg/image.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

std::string resource_path = "/home/bolty/ament_ws/src/semantic_segmentation/resource";
std::string model_path = "/perception_models/semantic_segmentation/pp_liteseg_infer_model/";
// 19 classes
std::unordered_map<int, std::string> cityscapes_labels = {
    {0, "road"}, {1, "sidewalk"},      {2, "building"},     {3, "wall"},       {4, "fence"},
    {5, "pole"}, {6, "traffic light"}, {7, "traffic sign"}, {8, "vegetation"}, {9, "terrain"},
    {10, "sky"}, {11, "person"},       {12, "rider"},       {13, "car"},       {14, "truck"},
    {15, "bus"}, {16, "train"},        {17, "motorcycle"},  {18, "bicycle"}};

std::unordered_map<int, std::tuple<int, int, int>> cityscapes_label_colormap = {
    {0, std::make_tuple(128, 64, 128)},  {1, std::make_tuple(244, 35, 232)},
    {2, std::make_tuple(70, 70, 70)},    {3, std::make_tuple(102, 102, 156)},
    {4, std::make_tuple(190, 153, 153)}, {5, std::make_tuple(153, 153, 153)},
    {6, std::make_tuple(250, 170, 30)},  {7, std::make_tuple(220, 220, 0)},
    {8, std::make_tuple(107, 142, 35)},  {9, std::make_tuple(152, 251, 152)},
    {10, std::make_tuple(70, 130, 180)}, {11, std::make_tuple(220, 20, 60)},
    {12, std::make_tuple(255, 0, 0)},    {13, std::make_tuple(0, 0, 142)},
    {14, std::make_tuple(0, 0, 70)},     {15, std::make_tuple(0, 60, 100)},
    {16, std::make_tuple(0, 80, 100)},   {17, std::make_tuple(0, 0, 230)},
    {18, std::make_tuple(119, 11, 32)}};

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
  std::string FLAGS_dynamic_shape_path = resource_path + "/dynamic_shape.pbtxt";
  std::string FLAGS_img_path = resource_path + "/cityscapes_demo.png";
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

  void hwc_img_2_chw_data(const cv::Mat& hwc_img, float* data) {
    int rows = hwc_img.rows;
    int cols = hwc_img.cols;
    int chs = hwc_img.channels();
    for (int i = 0; i < chs; ++i) {
      cv::extractChannel(hwc_img, cv::Mat(rows, cols, CV_32FC1, data + i * rows * cols), i);
    }
  }

  // Function to resize an image while maintaining aspect ratio
  cv::Mat resizeImageAspectRatio(const cv::Mat& originalImage, int maxWidth, int maxHeight) {
    int originalWidth = originalImage.cols;
    int originalHeight = originalImage.rows;
    double originalAspectRatio = (double)originalWidth / originalHeight;

    int newWidth, newHeight;
    double maxAspectRatio = (double)maxWidth / maxHeight;

    if (originalAspectRatio > maxAspectRatio) {
      // Width is the limiting factor
      newWidth = maxWidth;
      newHeight = static_cast<int>(maxWidth / originalAspectRatio);
    } else {
      // Height is the limiting factor
      newHeight = maxHeight;
      newWidth = static_cast<int>(maxHeight * originalAspectRatio);
    }

    cv::Mat resizedImage;
    cv::resize(originalImage, resizedImage, cv::Size(newWidth, newHeight));

    return resizedImage;
  }

  cv::Mat resizeWithPadding(const cv::Mat& originalImage, int targetWidth, int targetHeight) {
    int originalWidth = originalImage.cols;
    int originalHeight = originalImage.rows;

    double targetRatio = (double)targetWidth / targetHeight;
    double originalRatio = (double)originalWidth / originalHeight;

    int newWidth, newHeight;

    if (originalRatio > targetRatio) {
      // Original is wider. Fit to width and pad height.
      newWidth = targetWidth;
      newHeight =
          static_cast<int>(originalHeight * (static_cast<double>(targetWidth) / originalWidth));
    } else {
      // Original is taller. Fit to height and pad width.
      newHeight = targetHeight;
      newWidth =
          static_cast<int>(originalWidth * (static_cast<double>(targetHeight) / originalHeight));
    }

    cv::Mat resizedImage;
    cv::resize(originalImage, resizedImage, cv::Size(newWidth, newHeight));

    int top = (targetHeight - newHeight) / 2;
    int bottom = targetHeight - newHeight - top;
    int left = (targetWidth - newWidth) / 2;
    int right = targetWidth - newWidth - left;

    cv::Mat paddedImage;
    cv::copyMakeBorder(resizedImage, paddedImage, top, bottom, left, right, cv::BORDER_CONSTANT,
                       cv::Scalar(0, 0, 0));

    return paddedImage;
  }

  cv::Mat resizeFromCenter(const cv::Mat& originalImage, int targetWidth, int targetHeight) {
    int originalWidth = originalImage.cols;
    int originalHeight = originalImage.rows;

    // Calculate the new height maintaining the aspect ratio
    double targetRatio = (double)targetWidth / targetHeight;
    int newHeight = static_cast<int>(originalWidth / targetRatio);

    // Calculate the cropping area
    int startY = (originalHeight - newHeight) / 2;

    // Crop the image from the center
    cv::Rect roi(0, startY, originalWidth, newHeight);
    cv::Mat croppedImage = originalImage(roi);

    // Resize the cropped image
    cv::Mat resizedImage;
    cv::resize(croppedImage, resizedImage, cv::Size(targetWidth, targetHeight));

    return resizedImage;
  }

  void process_image(cv::Mat& img, const YamlConfig& yaml_config) {
    // cv::Mat img = cv::imread(FLAGS_img_path, cv::IMREAD_COLOR);
    // img = resizeWithPadding(img, 2048, 1024);
    img = resizeFromCenter(img, 2048, 1024);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    if (yaml_config.is_resize) {
      cv::resize(img, img, cv::Size(yaml_config.resize_width, yaml_config.resize_height));
    }
    if (yaml_config.is_normalize) {
      img.convertTo(img, CV_32F, 1.0 / 255, 0);
      img = (img - 0.5) / 0.5;
    }
  }

  cv::Mat read_process_image(const YamlConfig& yaml_config) {
    cv::Mat img = cv::imread(FLAGS_img_path, cv::IMREAD_COLOR);
    cv::cvtColor(img, img, cv::COLOR_BGR2RGB);
    if (yaml_config.is_resize) {
      cv::resize(img, img, cv::Size(yaml_config.resize_width, yaml_config.resize_height));
    }
    if (yaml_config.is_normalize) {
      img.convertTo(img, CV_32F, 1.0 / 255, 0);
      img = (img - 0.5) / 0.5;
    }
    return img;
  }

  SemanticSegmentationNode() : Node("semantic_segmentation_node"), count_(0) {
    // Get the image topic parameter from the launch file)
    std::string image_topic = this->declare_parameter("input_topic", "/camera/center/image_color");
    image_topic = this->get_parameter("input_topic").as_string();
    RCLCPP_INFO(this->get_logger(), "subscribing to image_topic: %s", image_topic.c_str());

    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    yaml_config_ = load_yaml(resource_path + "/deploy.yaml");

    // Print out the yaml config
    RCLCPP_INFO(this->get_logger(), "model_file: %s", yaml_config_.model_file.c_str());
    RCLCPP_INFO(this->get_logger(), "params_file: %s", yaml_config_.params_file.c_str());
    RCLCPP_INFO(this->get_logger(), "is_normalize: %d", yaml_config_.is_normalize);
    RCLCPP_INFO(this->get_logger(), "is_resize: %d", yaml_config_.is_resize);

    // Create the paddle predictor
    predictor_ = create_predictor(yaml_config_);

    // Subscribe to the image topic
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        image_topic, 10,
        std::bind(&SemanticSegmentationNode::image_callback, this, std::placeholders::_1));

    image_publisher_ =
        this->create_publisher<sensor_msgs::msg::Image>("semantic_segmentation_image", 10);
  }

 private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    RCLCPP_INFO(this->get_logger(), "Received image");
    cv::Mat img;
    cv::Mat original_img;

    // Convert msg to a cv mat

    try {
      img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      original_img = img.clone();
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.",
                   msg->encoding.c_str());
      return;
    }

    // Process the image according to yaml config
    RCLCPP_INFO(this->get_logger(), "Image shape before process: %d, %d, %d", img.rows, img.cols,
                img.channels());
    process_image(img, yaml_config_);
    // img = read_process_image(yaml_config_);

    int rows = img.rows;
    int cols = img.cols;
    int chs = img.channels();
    RCLCPP_INFO(this->get_logger(), "Image shape after process: %d, %d, %d", rows, cols, chs);
    std::vector<float> input_data(1 * chs * rows * cols, 0.0f);
    hwc_img_2_chw_data(img, input_data.data());

    // Set input
    auto input_names = predictor_->GetInputNames();
    auto input_t = predictor_->GetInputHandle(input_names[0]);
    std::vector<int> input_shape = {1, chs, rows, cols};
    input_t->Reshape(input_shape);
    input_t->CopyFromCpu(input_data.data());

    // Start timing
    auto start = std::chrono::high_resolution_clock::now();

    // Run
    predictor_->Run();

    // Stop timing
    auto end = std::chrono::high_resolution_clock::now();

    // Calculate elapsed time
    std::chrono::duration<double, std::milli> elapsed = end - start;

    // Get output
    auto output_names = predictor_->GetOutputNames();
    auto output_t = predictor_->GetOutputHandle(output_names[0]);
    std::vector<int> output_shape = output_t->shape();
    int out_num =
        std::accumulate(output_shape.begin(), output_shape.end(), 1, std::multiplies<int>());
    std::vector<int32_t> out_data(out_num);
    output_t->CopyToCpu(out_data.data());

    // Get pseudo image
    std::vector<uint8_t> out_data_u8(out_num);
    cv::Mat out_gray_img(output_shape[1], output_shape[2], CV_8UC1, out_data_u8.data());
    cv::Mat colored_img;
    // apply cityscapes_labels_colormap to img
    cv::cvtColor(out_gray_img, colored_img, cv::COLOR_GRAY2BGR);
    for (int i = 0; i < output_shape[1]; i++) {
      for (int j = 0; j < output_shape[2]; j++) {
        int label = out_data[i * output_shape[2] + j];
        if (cityscapes_labels.find(label) != cityscapes_labels.end()) {
          auto [r, g, b] = cityscapes_label_colormap[label];
          colored_img.at<cv::Vec3b>(i, j) = cv::Vec3b(b, g, r);
        }
      }
    }

    if (count_ < 1000) {
      // write the source image
      cv::imwrite("/home/bolty/ament_ws/src/semantic_segmentation/test/src_img" +
                      std::to_string(count_) + ".jpg",
                  original_img);
      // write the segmentation image
      cv::imwrite("/home/bolty/ament_ws/src/semantic_segmentation/test/out_img" +
                      std::to_string(count_) + ".jpg",
                  colored_img);
      RCLCPP_INFO(this->get_logger(), "Image written to /tmp/out_img%d.jpg, inference took %f ms",
                  count_, elapsed.count());
      count_++;
    }

    // Publish color image to topic
    sensor_msgs::msg::Image::SharedPtr out_img_msg =
        cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", colored_img).toImageMsg();

    image_publisher_->publish(*out_img_msg);
  }
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
  int count_;
  std::shared_ptr<paddle_infer::Predictor> predictor_;
  YamlConfig yaml_config_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SemanticSegmentationNode>());
  rclcpp::shutdown();
  return 0;
}