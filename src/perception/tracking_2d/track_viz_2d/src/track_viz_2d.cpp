#include "track_viz_2d.hpp"

track_viz_2d::track_viz_2d() : Node("track_viz_2d") {
  initializeParams();

  // Subscribers
  dets_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    detections_topic_, 10,
    std::bind(&track_viz_2d::detectionsCallback, this, std::placeholders::_1)
  );
  trks_sub_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    track_topic_, 10,
    std::bind(&track_viz_2d::tracksCallback, this, std::placeholders::_1)
  );
  image_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
    image_sub_topic_, 10,
    std::bind(&track_viz_2d::imageCallback, this, std::placeholders::_1)
  );

  // Publishers
  image_pub_ =
      this->create_publisher<sensor_msgs::msg::Image>(image_pub_topic_, 10);

}

void track_viz_2d::initializeParams() {
  // Declare parameters
  this->declare_parameter<std::string>("detections_topic", "/camera_object_detections");
  this->declare_parameter<std::string>("track_topic", "/tracked_boxes");
  this->declare_parameter<std::string>("image_sub_topic", "/CAM_FRONT/image_rect_compressed");
  this->declare_parameter<std::string>("image_pub_topic", "/annotated_image");
  this->declare_parameter<std::string>("camera_frame", "CAM_FRONT");
  this->declare_parameter<std::string>("color_dets", "blue");
  this->declare_parameter<std::string>("color_trks", "red");
  this->declare_parameter<int>("bbox_line_width", 5);

  // Get parameters
  bool params_ok = true;

  params_ok &= this->get_parameter("detections_topic", detections_topic_);  // string
  params_ok &= this->get_parameter("track_topic", track_topic_);            // string
  params_ok &= this->get_parameter("image_sub_topic", image_sub_topic_);    // string
  params_ok &= this->get_parameter("image_pub_topic", image_pub_topic_);    // string
  params_ok &= this->get_parameter("camera_frame", camera_frame_);          // string
  params_ok &= this->get_parameter("color_dets", color_dets_);              // string
  params_ok &= this->get_parameter("color_trks", color_trks_);              // string
  params_ok &= this->get_parameter("bbox_line_width", bbox_line_width_);    // int

  color_map_ = {
    {"red", cv::Scalar(0, 0, 255)},
    {"green", cv::Scalar(0, 255, 0)},
    {"blue", cv::Scalar(255, 0, 0)},
    {"white", cv::Scalar(255, 255, 255)},
    {"black", cv::Scalar(0, 0, 0)},
  };

  default_color_ = cv::Scalar(255, 0, 0);

  latest_image_ = nullptr;
  latest_dets_ = nullptr;

  if (!params_ok) RCLCPP_WARN(this->get_logger(), "One or more parameters could not be initialized");
  else RCLCPP_INFO(this->get_logger(), "Parameters initialized");
}


// Get color from color_map_
cv::Scalar track_viz_2d::colorLookup(std::string color) {
  auto it = color_map_.find(color);
  if (it != color_map_.end()) return it->second;
  else {
    RCLCPP_WARN(this->get_logger(), "Color '%s' not found, using default", color.c_str());
    return default_color_;
  }
}


// Draw a set of boxes onto the image
void track_viz_2d::drawBoxes(
  cv::Mat &image,
  const std::vector<vision_msgs::msg::Detection2D> &dets,
  const cv::Scalar &color
) {
  for (const auto &det : dets) {
    int x = static_cast<int>(det.bbox.center.position.x - det.bbox.size_x/2);
    int y = static_cast<int>(det.bbox.center.position.y - det.bbox.size_y/2);
    int width = static_cast<int>(det.bbox.size_x);
    int height = static_cast<int>(det.bbox.size_y);

    cv::Rect rect(x, y, width, height);

    cv::rectangle(image, rect, color, bbox_line_width_);
  }
}


// Attempt box drawing
void track_viz_2d::tryDraw(
  cv::Mat &decoded_img,
  const vision_msgs::msg::Detection2DArray::SharedPtr latest_trks_
) {
  if (decoded_img.empty() || !latest_dets_) {
    RCLCPP_WARN(this->get_logger(), "Missing image or detections");
    return;
  }

  const std::vector<vision_msgs::msg::Detection2D> dets = latest_dets_->detections;
  const std::vector<vision_msgs::msg::Detection2D> trks = latest_trks_->detections;

  drawBoxes(decoded_img, dets, colorLookup(color_dets_));
  drawBoxes(decoded_img, trks, colorLookup(color_trks_));

  std_msgs::msg::Header header;
  header.frame_id = camera_frame_;
  header.stamp = this->now();

  cv_bridge::CvImage cv_img(header, "bgr8", decoded_img);

  RCLCPP_INFO(this->get_logger(), "Publishing image...");
  image_pub_->publish(*(cv_img.toImageMsg()));
}


void track_viz_2d::imageCallback(
  const sensor_msgs::msg::CompressedImage::SharedPtr msg
) {
  latest_image_ = msg;
}


void track_viz_2d::detectionsCallback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg
) {
  latest_dets_ = msg;
}


void track_viz_2d::tracksCallback(
  const vision_msgs::msg::Detection2DArray::SharedPtr msg
) {
  if (!latest_image_) {
    RCLCPP_WARN(this->get_logger(), "Missing image");
    return;
  }

  try {
    // Attempt to decode compressed image
    const std::vector<uint8_t> &raw_data = latest_image_->data;
    cv::Mat decoded = cv::imdecode(raw_data, cv::IMREAD_COLOR);
    if (decoded.empty()) {
      RCLCPP_WARN(this->get_logger(), "Failed to decode compressed image");
      return;
    }
    tryDraw(decoded, msg);
  } catch (const cv::Exception &e) {
    RCLCPP_ERROR(this->get_logger(), "OpenCV exception: %s", e.what());
  }
}


int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<track_viz_2d>());
  rclcpp::shutdown();
  return 0;
}
