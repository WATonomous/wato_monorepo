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

#include <cmath>
#include <filesystem>
#include <functional>
#include <future>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <semvecnet/semvecnet.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "lanelets/lanelets_core.hpp"

namespace wato::perception::lanelets
{

  namespace
  {
    constexpr int kLogThrottleMs = 5000;
    constexpr auto kRawImageTopic = "input_multi_image_raw";
    constexpr auto kCompressedImageTopic = "input_multi_image_compressed";
    constexpr auto kCameraInfoTopic = "input_multi_camera_info";
    constexpr auto kLidarTopic = "input_cloud";
    constexpr auto kLaneletsTopic = "output_lanelets";

    // Like BEVFusion, index by frame ID and use the configured camera order for both
    // images and calibration, regardless of their order in the incoming messages.
    template <class Message>
    std::vector<const Message *> orderCameras(
        const std::vector<Message> &messages, const std::vector<std::string> &camera_names)
    {
      std::unordered_map<std::string, const Message *> by_frame;
      by_frame.reserve(messages.size());
      for (const auto &msg : messages)
      {
        if (!by_frame.emplace(msg.header.frame_id, &msg).second)
        {
          throw std::invalid_argument("Duplicate camera frame " + msg.header.frame_id);
        }
      }
      std::vector<const Message *> ordered;
      ordered.reserve(camera_names.size());
      for (const auto &name : camera_names)
      {
        const auto it = by_frame.find(name);
        if (it == by_frame.end())
        {
          throw std::invalid_argument("Missing camera frame " + name);
        }
        ordered.push_back(it->second);
      }
      return ordered;
    }

    cv::Mat decompressImage(const sensor_msgs::msg::CompressedImage &image)
    {
      cv::Mat rgb;
      const cv::Mat bgr = cv::imdecode(image.data, cv::IMREAD_COLOR);
      if (!bgr.empty())
      {
        cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
      }
      return rgb;
    }
  } // namespace

  LaneletsNode::LaneletsNode(const rclcpp::NodeOptions &options)
      : Node("lanelets_node", options)
  {
    const bool use_compressed = declare_parameter<bool>("use_compressed_images", true);
    camera_names_ = declare_parameter<std::vector<std::string>>(
        "camera_names",
        {"camera_pano_nn", "camera_pano_ne", "camera_pano_nw", "camera_pano_ss", "camera_pano_sw", "camera_pano_se"});
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    ego_frame_ = declare_parameter<std::string>("ego_frame", "base_link");
    const int image_width = declare_parameter<int>("image_width", 1280);
    const int image_height = declare_parameter<int>("image_height", 1024);
    const int max_points = declare_parameter<int>("max_points", 300000);
    const int sync_queue_size = declare_parameter<int>("sync_queue_size", 10);
    const double sync_max_time_diff_ms = declare_parameter<double>("sync_max_time_diff_ms", 200.0);
    const auto hrnet_plan = declare_parameter<std::string>(
        "hrnet_plan", "/opt/watonomous/models/semvecnet/hrnet_ocrnet_mapillary.plan");
    const auto semvecnet_onnx =
        declare_parameter<std::string>("semvecnet_onnx", "/opt/watonomous/models/semvecnet/semvecnet.onnx");
    if (
        camera_names_.empty() ||
        std::unordered_set<std::string>(camera_names_.begin(), camera_names_.end()).size() != camera_names_.size() ||
        map_frame_.empty() || ego_frame_.empty() || sync_queue_size <= 0 ||
        !std::isfinite(sync_max_time_diff_ms) || sync_max_time_diff_ms <= 0.0)
    {
      throw std::invalid_argument("Invalid camera names, map/ego frame, or synchronization parameters");
    }
    core_ = std::make_unique<LaneletsCore>(image_width, image_height, max_points);
    for (const auto &model_path : {hrnet_plan, semvecnet_onnx})
    {
      if (!std::filesystem::is_regular_file(model_path))
      {
        throw std::runtime_error("SemVecNet model file not found: " + model_path);
      }
    }
    semvecnet_ = std::make_unique<semvecnet::SemVecNet>(hrnet_plan, semvecnet_onnx);
    lanelets_pub_ = create_publisher<lanelet_msgs::msg::MapVisualization>(kLaneletsTopic, 10);
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);

    const auto sensor_qos = rclcpp::SensorDataQoS().keep_last(sync_queue_size);
    camera_info_sub_ = create_subscription<MultiCameraInfo>(
        kCameraInfoTopic, sensor_qos, [this](MultiCameraInfo::ConstSharedPtr msg)
        { cached_camera_info_ = msg; });
    lidar_sub_.subscribe(this, kLidarTopic, sensor_qos.get_rmw_qos_profile());
    const auto max_interval = rclcpp::Duration::from_seconds(sync_max_time_diff_ms / 1000.0);
    if (use_compressed)
    {
      compressed_sub_.subscribe(this, kCompressedImageTopic, sensor_qos.get_rmw_qos_profile());
      compressed_sync_ = std::make_unique<message_filters::Synchronizer<CompressedSyncPolicy>>(
          CompressedSyncPolicy(sync_queue_size), compressed_sub_, lidar_sub_);
      compressed_sync_->setMaxIntervalDuration(max_interval);
      compressed_sync_->registerCallback(
          std::bind(&LaneletsNode::syncedCompressedCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    else
    {
      raw_sub_.subscribe(this, kRawImageTopic, sensor_qos.get_rmw_qos_profile());
      raw_sync_ = std::make_unique<message_filters::Synchronizer<RawSyncPolicy>>(
          RawSyncPolicy(sync_queue_size), raw_sub_, lidar_sub_);
      raw_sync_->setMaxIntervalDuration(max_interval);
      raw_sync_->registerCallback(
          std::bind(&LaneletsNode::syncedRawCallback, this, std::placeholders::_1, std::placeholders::_2));
    }
    RCLCPP_INFO(
        get_logger(),
        "Lanelets preprocessing ready: %zu cameras, %s images, sync window %.1f ms, map frame '%s'",
        camera_names_.size(),
        use_compressed ? "compressed" : "raw",
        sync_max_time_diff_ms,
        map_frame_.c_str());
  }

  LaneletsNode::~LaneletsNode() = default;

  void LaneletsNode::syncedRawCallback(
      const MultiImage::ConstSharedPtr &images, const PointCloud::ConstSharedPtr &cloud)
  {
    try
    {
      const auto ordered = orderCameras(images->images, camera_names_);
      std::vector<cv::Mat> rgb_images;
      rgb_images.reserve(ordered.size());
      for (const auto *image : ordered)
      {
        // With RGB8 this can view the message storage; images stays alive through processFrame.
        rgb_images.push_back(cv_bridge::toCvShare(*image, images, sensor_msgs::image_encodings::RGB8)->image);
      }
      processFrame(rgb_images, *cloud);
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "Skipping synced raw frame: %s", e.what());
    }
  }

  void LaneletsNode::syncedCompressedCallback(
      const MultiImageCompressed::ConstSharedPtr &images, const PointCloud::ConstSharedPtr &cloud)
  {
    try
    {
      const auto ordered = orderCameras(images->images, camera_names_);
      // Decode independently, then collect in the original camera order.
      std::vector<std::future<cv::Mat>> decode_futures;
      decode_futures.reserve(ordered.size());
      for (const auto *image : ordered)
      {
        decode_futures.push_back(std::async(std::launch::async, [image]()
                                            { return decompressImage(*image); }));
      }
      std::vector<cv::Mat> rgb_images;
      rgb_images.reserve(ordered.size());
      for (auto &future : decode_futures)
      {
        rgb_images.push_back(future.get());
      }
      processFrame(rgb_images, *cloud);
    }
    catch (const std::exception &e)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "Skipping synced compressed frame: %s", e.what());
    }
  }

  void LaneletsNode::processFrame(std::vector<cv::Mat> &rgb_images, const PointCloud &cloud)
  {
    if (!cached_camera_info_)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "Waiting for multi-camera calibration");
      return;
    }
    const auto ordered_info = orderCameras(cached_camera_info_->camera_infos, camera_names_);
    if (cloud.header.frame_id.empty() || rclcpp::Time(cloud.header.stamp).nanoseconds() <= 0)
    {
      throw std::invalid_argument("Expected a stamped LiDAR cloud with a frame ID");
    }
    const rclcpp::Time stamp(cloud.header.stamp);
    const auto lidar_to_map =
        LaneletsCore::flattenTransform(tf_buffer_->lookupTransform(map_frame_, cloud.header.frame_id, stamp));
    const auto ego_to_map =
        LaneletsCore::flattenTransform(tf_buffer_->lookupTransform(map_frame_, ego_frame_, stamp));

    // Keep both vectors in camera_names order. rgb_images owns/views the pixels until this callback returns.
    std::vector<const unsigned char *> camera_images;
    std::vector<semvecnet::semantic_projection::CameraCalibration> calibrations;
    camera_images.reserve(rgb_images.size());
    calibrations.reserve(rgb_images.size());
    for (std::size_t i = 0; i < rgb_images.size(); ++i)
    {
      auto &image = rgb_images[i];
      const auto &info = *ordered_info[i];
      if (
          info.width != static_cast<unsigned>(image.cols) || info.height != static_cast<unsigned>(image.rows) ||
          !core_->validateAndNormalizeImage(image))
      {
        throw std::invalid_argument("Invalid RGB image or CameraInfo dimensions for " + info.header.frame_id);
      }
      camera_images.push_back(image.data);
      const auto calibration = LaneletsCore::computeCameraCalibration(
          info, tf_buffer_->lookupTransform(info.header.frame_id, cloud.header.frame_id, stamp));
      calibrations.push_back({calibration.intrinsics, calibration.lidar_to_camera});
    }

    const auto lidar_points = core_->processLidar(cloud);
    const int num_points = static_cast<int>(lidar_points.size() / LaneletsCore::kLidarFeatures);
    if (num_points == 0)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "No valid LiDAR points after preprocessing");
      return;
    }
    const int num_cameras = static_cast<int>(camera_images.size());
    const int image_width = core_->imageWidth();
    const int image_height = core_->imageHeight();

    // Copy the binary16 payload into the vendor type without reinterpreting unrelated objects.
    std::vector<nvtype::half> semvecnet_points;
    semvecnet_points.reserve(lidar_points.size());
    for (const auto bits : lidar_points)
    {
      semvecnet_points.push_back(nvtype::half{bits});
    }

    semvecnet::Polylines polylines;
    try
    {
      polylines = semvecnet_->forward(
          camera_images.data(), semvecnet_points.data(), num_points, num_cameras, image_width, image_height,
          calibrations, lidar_to_map, ego_to_map);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), kLogThrottleMs, "SemVecNet forward failed: %s", e.what());
      return;
    }
    RCLCPP_DEBUG_THROTTLE(
        get_logger(), *get_clock(), kLogThrottleMs, "SemVecNet returned %zu polylines", polylines.size());
    const auto lanelets = generateLanelets(polylines, ego_to_map);
    if (!lanelets.empty())
    {
      publishLanelets(lanelets, stamp);
    }
  }

  lanelet::Lanelets LaneletsNode::generateLanelets(
      const semvecnet::Polylines & /*polylines*/, const Matrix4 & /*ego_to_map*/)
  {
    // TODO(lanelets): Pair boundaries, construct Lanelet2 geometry, and transform it into map_frame_.
    return {};
  }

  void LaneletsNode::publishLanelets(const lanelet::Lanelets &lanelets, const rclcpp::Time &stamp)
  {
    lanelet_msgs::msg::MapVisualization msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = map_frame_;
    msg.lanelets.reserve(lanelets.size());
    for (const auto &lanelet : lanelets)
    {
      msg.lanelets.push_back(LaneletsCore::toLaneletMsg(lanelet));
    }
    lanelets_pub_->publish(msg);
  }

} // namespace wato::perception::lanelets

RCLCPP_COMPONENTS_REGISTER_NODE(wato::perception::lanelets::LaneletsNode)
