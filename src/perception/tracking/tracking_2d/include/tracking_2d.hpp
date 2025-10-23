#ifndef TRACKING_2D_HPP
#define TRACKING_2D_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vision_msgs/msg/detection2_d_array.hpp>

#include <ByteTrack/BYTETracker.h>


class tracking_2d : public rclcpp::Node {
 public:
  tracking_2d();

 private:
  // Callback functions
  void detectionsCallback(const vision_msgs::msg::Detection2DArray::SharedPtr msg);
  void imageCallback(sensor_msgs::msg::Image::SharedPtr msg);

  // Helper functions
  std::vector<byte_track::Object> detsToObjects(const vision_msgs::msg::Detection2DArray::SharedPtr dets);
  vision_msgs::msg::Detection2DArray STracksToDets(const std::vector<byte_track::BYTETracker::STrackPtr> &strk_ptrs, const std_msgs::msg::Header &header);

  // Subscribers
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr
      dets_sub_;

  // Publishers
  rclcpp::Publisher<vision_msgs::msg::Detection2DArray>::SharedPtr tracked_dets_pub_;

  // Parameters
  void initializeParams();
  std::string detections_topic_;
  std::string track_topic_;

  int frame_rate_;
  int track_buffer_;
  float track_thresh_;
  float high_thresh_;
  float match_thresh_;

  std::unique_ptr<byte_track::BYTETracker> tracker_;

};

#endif
