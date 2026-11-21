// Copyright (c) 2025-present WATonomous. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "lanelets/lanelets_node.hpp"

#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace wato::perception::lanelets
{

namespace
{
constexpr int kLogThrottleMs = 2000;
}  // namespace

LaneletsNode::LaneletsNode(const rclcpp::NodeOptions & options)
: Node("lanelets_node", options)
{
  const bool use_compressed = declare_parameter<bool>("use_compressed_images", true);

  const auto sensor_qos = rclcpp::SensorDataQoS();

  if (use_compressed) {
    compressed_image_sub_ = create_subscription<sensor_msgs::msg::CompressedImage>(
      kImageTopic, sensor_qos, [this](const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg) {
        onCompressedImage(msg);
      });
  } else {
    raw_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      kImageTopic, sensor_qos, [this](const sensor_msgs::msg::Image::ConstSharedPtr & msg) { onRawImage(msg); });
  }

  camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
    kCameraInfoTopic, sensor_qos, [this](const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg) {
      onCameraInfo(msg);
    });

  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
    kCloudTopic, sensor_qos, [this](const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg) { onCloud(msg); });

  RCLCPP_INFO(
    get_logger(),
    "Lanelets node started. image: %s (%s), camera_info: %s, cloud: %s",
    (use_compressed ? compressed_image_sub_->get_topic_name() : raw_image_sub_->get_topic_name()),
    use_compressed ? "compressed" : "raw",
    camera_info_sub_->get_topic_name(),
    cloud_sub_->get_topic_name());
}

void LaneletsNode::onCompressedImage(const sensor_msgs::msg::CompressedImage::ConstSharedPtr & msg)
{
  const cv::Mat bgr = cv::imdecode(msg->data, cv::IMREAD_COLOR);
  if (bgr.empty()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), kLogThrottleMs, "Failed to decode compressed image (format '%s')",
      msg->format.c_str());
    return;
  }
  storeImage(msg->header, bgr);
}

void LaneletsNode::onRawImage(const sensor_msgs::msg::Image::ConstSharedPtr & msg)
{
  try {
    // toCvCopy so the stored cv::Mat owns its memory independently of the message.
    storeImage(msg->header, cv_bridge::toCvCopy(msg, "bgr8")->image);
  } catch (const cv_bridge::Exception & e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "cv_bridge failed: %s", e.what());
  }
}

void LaneletsNode::storeImage(const std_msgs::msg::Header & header, const cv::Mat & bgr)
{
  {
    std::lock_guard<std::mutex> lock(mutex_);
    latest_image_ = bgr;
    latest_image_header_ = header;
  }
  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kLogThrottleMs, "Camera: %dx%d image, frame '%s', stamp %.3f",
    bgr.cols, bgr.rows, header.frame_id.c_str(), rclcpp::Time(header.stamp).seconds());
}

void LaneletsNode::onCameraInfo(const sensor_msgs::msg::CameraInfo::ConstSharedPtr & msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_camera_info_) {
    RCLCPP_INFO(
      get_logger(), "Camera info: %ux%u, frame '%s', fx=%.1f fy=%.1f cx=%.1f cy=%.1f",
      msg->width, msg->height, msg->header.frame_id.c_str(), msg->k[0], msg->k[4], msg->k[2], msg->k[5]);
  }
  latest_camera_info_ = msg;
}

void LaneletsNode::onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & msg)
{
  std::string fields;
  for (const auto & f : msg->fields) {
    fields += (fields.empty() ? "" : ",") + f.name;
  }

  std::string image_dt = "n/a (no image yet)";
  {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!latest_image_.empty()) {
      const double dt = (rclcpp::Time(msg->header.stamp) - rclcpp::Time(latest_image_header_.stamp)).seconds();
      image_dt = std::to_string(dt) + " s";
    }
  }

  RCLCPP_INFO_THROTTLE(
    get_logger(), *get_clock(), kLogThrottleMs,
    "LiDAR: %u points, frame '%s', fields [%s], stamp %.3f, latest image dt %s",
    msg->width * msg->height, msg->header.frame_id.c_str(), fields.c_str(),
    rclcpp::Time(msg->header.stamp).seconds(), image_dt.c_str());

  // TODO(lanelets): run lanelet generation on (cloud, latest_image_, latest_camera_info_) here.
}

}  // namespace wato::perception::lanelets

RCLCPP_COMPONENTS_REGISTER_NODE(wato::perception::lanelets::LaneletsNode)
