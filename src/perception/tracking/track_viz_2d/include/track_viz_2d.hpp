#ifndef TRACK_VIZ_2D_HPP
#define TRACK_VIZ_2D_HPP

#include <rclcpp/rclcpp.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/header.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>


class track_viz_2d : public rclcpp::Node {
 public:
  track_viz_2d();

 private:
  // Callback functions
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void tracksCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void imageCallback(const sensor_msgs::msg::CompressedImage::SharedPtr msg);

  // Helper functions
  cv::Scalar colorLookup(std::string color);
  void drawBoxes(
    cv::Mat &image,
    const std::vector<vision_msgs::msg::Detection2D> &dets,
    const cv::Scalar &color
  );
  void tryDraw(
    cv::Mat &decoded_img,
    const vision_msgs::msg::Detection2DArray::SharedPtr latest_trks_
  );

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr dets_sub_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr trks_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr image_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;

  // Parameters
  void initializeParams();
  std::string detections_topic_;
  std::string track_topic_;
  std::string image_sub_topic_;
  std::string image_pub_topic_;
  std::string camera_frame_;
  std::string color_dets_;
  std::string color_trks_;
  int bbox_line_width_;

  std::unordered_map<std::string, cv::Scalar> color_map_;
  cv::Scalar default_color_;

  sensor_msgs::msg::CompressedImage::SharedPtr latest_image_;
  vision_msgs::msg::Detection2DArray::SharedPtr latest_dets_;
};

#endif
