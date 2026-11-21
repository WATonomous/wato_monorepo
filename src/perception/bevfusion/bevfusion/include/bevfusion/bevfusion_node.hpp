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

#pragma once

// clang-format off
// C system headers (.h)
// C++ system headers
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>
// Other
#include <diagnostic_updater/diagnostic_updater.hpp>
#include <diagnostic_updater/publisher.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <isaac_ros_managed_nitros/managed_nitros_subscriber.hpp>
#include <isaac_ros_nitros_image_type/nitros_image_view.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vision_msgs/msg/detection3_d_array.hpp>

#include "bevfusion/bevfusion_core.hpp"
#include "bevfusion/camera_matcher.hpp"
// clang-format on

namespace wato::perception::bevfusion
{

/**
 * @brief ROS 2 lifecycle composable node for BEVFusion
 *
 * Each camera is subscribed on its own topic: GPU (NITROS) images on the car, JPEGs from bags. For
 * every LiDAR scan the node picks, per camera, the frame nearest the moment the LiDAR swept that
 * camera's direction (CameraMatcher), then runs inference on a worker thread.
 */
class BEVFusionNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  BEVFusionNode() = delete;

  /**
   * @brief Construct the node and declare parameters.
   * @param options Node options for remapping, parameters, etc.
   */
  explicit BEVFusionNode(const rclcpp::NodeOptions & options);

  /// Stops the worker thread if the node is destroyed without a cleanup/shutdown transition.
  ~BEVFusionNode() override;

  // Default topic names (matches the remap "from" names in the launch file)
  static constexpr auto kLidarTopic = "/lidar_cc/velodyne_points";
  // Per-camera topics, subscribed as "/<camera_name>/<suffix>"
  static constexpr auto kCompressedImageTopicSuffix = "image_rect_compressed";
  static constexpr auto kNitrosImageTopicSuffix = "image_rect";
  static constexpr auto kCameraInfoTopicSuffix = "camera_info";
  static constexpr auto kOutputDetectionsTopic = "output_detections";
  static constexpr auto kOutputMarkersTopic = "output_markers";

  /**
   * @brief Lifecycle: configure parameters, core, QoS, and diagnostics.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS on configuration
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: create pubs/subs, the camera matcher and the inference worker.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS on activation
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: stop inputs and the worker, release held frames, deactivate publishers.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: destroy pubs/subs and diagnostics, release core.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Lifecycle: final shutdown and cleanup.
   * @param previous_state Previous lifecycle state
   * @return SUCCESS
   */
  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

private:
  // Aliases
  using CompressedImageMsg = sensor_msgs::msg::CompressedImage;
  using CameraInfoMsg = sensor_msgs::msg::CameraInfo;
  using PointCloud2Msg = sensor_msgs::msg::PointCloud2;
  using NitrosImageView = nvidia::isaac_ros::nitros::NitrosImageView;
  using NitrosImageMsg = NitrosImageView::BaseType;
  using NitrosImageSub = nvidia::isaac_ros::nitros::ManagedNitrosSubscriber<NitrosImageView>;

  /// Where camera images come from
  enum class CameraInput
  {
    kCompressed,  ///< Per-camera JPEG topics (bag replay); decoded on the CPU
    kNitros  ///< Per-camera NITROS images in GPU memory (the car); passed to the model without a copy
  };

  /// What each camera's frame is matched to
  enum class SyncTarget
  {
    kSweep,  ///< When the LiDAR beam swept that camera's viewing direction
    kScanStamp  ///< The scan stamp, for every camera
  };

  /// Which camera matrix is used for the intrinsics
  enum class IntrinsicsSource
  {
    kCameraMatrix,  ///< CameraInfo K (the raw, distorted camera)
    kProjection  ///< CameraInfo P (the camera after rectification)
  };

  /// One camera frame as held by the matcher; exactly one pointer is set
  struct CameraFrame
  {
    CompressedImageMsg::ConstSharedPtr jpeg;
    std::shared_ptr<const NitrosImageMsg> gpu;
  };

  using Matcher = CameraMatcher<CameraFrame, PointCloud2Msg::ConstSharedPtr>;
  using MatchedSet = Matcher::MatchedSet;

  /**
   * @brief Declare and populate core parameters from the parameter server.
   */
  void declareParameters();

  /// Declare a parameter unless it already is (a configure after cleanup keeps the values)
  template <typename T>
  void declareIfMissing(const std::string & name, const T & default_value)
  {
    if (!this->has_parameter(name)) {
      this->declare_parameter<T>(name, default_value);
    }
  }

  /**
   * @brief Declare and validate the camera input and matching parameters.
   * @throws std::invalid_argument for an unknown or out-of-range value
   */
  void declareSyncParameters();

  /**
   * @brief Store a JPEG frame from one camera in the matcher.
   * @param camera_idx Index of the camera in camera_names
   * @param msg Compressed image
   */
  void compressedImageCallback(size_t camera_idx, const CompressedImageMsg::ConstSharedPtr & msg);

  /**
   * @brief Store a NITROS (GPU) frame from one camera in the matcher.
   * @param camera_idx Index of the camera in camera_names
   * @param image_view View of the received NITROS image
   */
  void nitrosImageCallback(size_t camera_idx, const NitrosImageView & image_view);

  /**
   * @brief Add a LiDAR scan to the matcher, with one target time per camera.
   * @param lidar_msg PointCloud2 containing point cloud data
   */
  void lidarCallback(const PointCloud2Msg::ConstSharedPtr & lidar_msg);

  /**
   * @brief Cache one camera's info; computes calibration once every camera has reported.
   * @param camera_idx Index of the camera in camera_names
   * @param msg CameraInfo for that camera
   */
  void cameraInfoCallback(size_t camera_idx, const CameraInfoMsg::ConstSharedPtr & msg);

  /**
   * @brief Target time per camera for a scan: when the LiDAR swept each camera's direction.
   * @param lidar_msg Scan in firing order, stamped with its last packet
   * @return One target per camera, or an empty vector to use the scan stamp for every camera
   */
  std::vector<int64_t> scanTargets(const PointCloud2Msg & lidar_msg);

  /**
   * @brief Hand completed sets to the worker (the newest replaces one still waiting).
   * @param sets Sets completed by the matcher
   * @param epoch Matcher epoch the sets belong to (see matcher_epoch_)
   */
  void dispatchSets(std::vector<MatchedSet> & sets, uint64_t epoch);

  /**
   * @brief Count a matcher reset (a bag restarting) as a new epoch. Caller holds matcher_mutex_.
   * @param now_ns Current steady-clock time
   * @return The current epoch
   */
  uint64_t currentEpoch(int64_t now_ns);

  /**
   * @brief Warn (throttled) about cameras that have sent no frame for over a second, which makes every
   *        scan wait out its cap and be skipped. Silent for two seconds after activation or a reset.
   *        Caller holds matcher_mutex_.
   * @param now_ns Current steady-clock time
   */
  void warnAboutSilentCameras(int64_t now_ns);

  /// Remove the output topic's frequency diagnostic from the updater and destroy it
  void removeTopicDiagnostic();

  /**
   * @brief Arm the timer for the matcher's next deadline. Caller holds matcher_mutex_.
   * @param now_ns Current steady-clock time
   */
  void armDeadlineTimer(int64_t now_ns);

  /**
   * @brief Timer callback: let the matcher decide scans whose wait ran out.
   */
  void deadlineTimerCallback();

  /**
   * @brief Inference worker: waits for sets and processes them one at a time.
   */
  void workerLoop();

  /**
   * @brief Start the inference worker thread.
   */
  void startWorker();

  /**
   * @brief Stop and join the inference worker thread; drops a set still waiting.
   */
  void stopWorker();

  /**
   * @brief Destroy subscriptions and the deadline timer, then release every frame the matcher holds.
   */
  void stopInputs();

  /**
   * @brief Prepare camera images and run inference for one matched set (worker thread).
   * @param set Scan and the frame chosen for each camera
   */
  void processSet(const MatchedSet & set);

  /**
   * @brief Decode the set's JPEGs (in parallel) to RGB images at the configured size.
   * @param set Matched set with JPEG frames
   * @param rgb_images Output RGB images, one per camera
   * @return True if every image decoded and validated
   */
  bool decodeJpegImages(const MatchedSet & set, std::vector<cv::Mat> & rgb_images);

  /**
   * @brief Shared second half of frame processing: LiDAR prep, inference, publishing and statistics.
   * @param camera_images One pointer per camera to packed RGB8 pixels, in CPU or GPU memory
   * @param lidar_msg PointCloud2 containing point cloud data
   * @param stamp Stamp for the published detections (the scan stamp)
   * @param t_start Start time of processing this set
   * @param match_wait_ms Time from the scan's arrival until its set was complete
   */
  void runInference(
    const std::vector<const unsigned char *> & camera_images,
    const PointCloud2Msg::ConstSharedPtr & lidar_msg,
    const builtin_interfaces::msg::Time & stamp,
    std::chrono::steady_clock::time_point t_start,
    double match_wait_ms);

  /**
   * @brief Computes calibration matrices from camera info and TF extrinsics, then updates the core CUDA pipeline.
   * @details Called during node activation (`on_activate()`) if every camera's info is cached, and by
   *         `cameraInfoCallback()` once every camera has reported. Also computes each camera's azimuth
   *         in the LiDAR frame for sweep targets.
   */
  void computeCalibrationMatrices();

  /**
   * @brief Log total processed messages and average processing time.
   */
  void logStatistics() const;

  /**
   * @brief Build subscriber QoS from reliability/depth parameters.
   * @param reliability Reliability policy ("reliable" or "best_effort")
   * @param depth Queue depth
   * @return Configured QoS profile
   */
  rclcpp::QoS createSubscriberQoS(const std::string & reliability, int depth);

  /**
   * @brief Build publisher QoS from reliability/durability/depth parameters.
   * @param reliability Reliability policy ("reliable" or "best_effort")
   * @param durability Durability policy ("transient_local" or "volatile")
   * @param depth Queue depth
   * @return Configured QoS profile
   */
  rclcpp::QoS createPublisherQoS(const std::string & reliability, const std::string & durability, int depth);

  /**
   * @brief Diagnostic callback for node health/performance.
   * @param stat Diagnostic status wrapper to populate
   */
  void diagnosticCallback(diagnostic_updater::DiagnosticStatusWrapper & stat);

  /**
   * @brief Update counters/timers after processing.
   * @param time_taken Processing time in milliseconds
   */
  void updateStatistics(double time_taken);

  /**
   * @brief Tick topic diagnostics.
   * @param timestamp Timestamp from the processed detection array
   */
  void updateDiagnostics(const std_msgs::msg::Header::_stamp_type & timestamp);

  /**
   * @brief Decompress a CompressedImage to cv::Mat in BGR format.
   * @param compressed_img The compressed image message
   * @return Decompressed BGR cv::Mat (empty on error)
   */
  cv::Mat decompressImage(const sensor_msgs::msg::CompressedImage & compressed_img) const;

  /**
   * @brief Process a PointCloud2 message and extract LiDAR points.
   * @param lidar_msg The PointCloud2 message
   * @param lidar_data Output vector to flatten andstore extracted LiDAR points
   * @return True if the LiDAR data was processed successfully, false otherwise
   */
  bool processLidar(const sensor_msgs::msg::PointCloud2::ConstSharedPtr & lidar_msg, std::vector<float> & lidar_data);

  /**
   * @brief Validate and normalize an image.
   * @param img The image to validate and normalize
   * @param index The index of the image
   * @return True if the image was validated and normalized successfully, false otherwise
   */
  bool validateAndNormalizeImage(cv::Mat & img, size_t index);

  /**
   * @brief Validate and trim LiDAR data.
   * @param lidar_data The LiDAR data to validate and trim
   * @return True if the LiDAR data was validated and trimmed successfully, false otherwise
   */
  bool validateAndTrimLidar(std::vector<float> & lidar_data);

  /**
   * @brief Create a Detection3DArray from bounding boxes and timestamp.
   * @param bboxes Vector of BoundingBox objects
   * @param stamp Timestamp for the detections
   * @return Detection3DArray containing 3D bounding boxes and confidence scores
   */
  vision_msgs::msg::Detection3DArray createDetections3D(
    const std::vector<BoundingBox> & bboxes, const builtin_interfaces::msg::Time & stamp) const;

  /**
   * @brief Create a MarkerArray from a Detection3DArray.
   * @param detections_3d The Detection3DArray containing bounding boxes and confidence scores
   * @return MarkerArray containing visualization markers for the bounding boxes
   */
  visualization_msgs::msg::MarkerArray createMarkers(const vision_msgs::msg::Detection3DArray & detections_3d) const;

  /// Steady-clock time in nanoseconds (arrival and deadline times for the matcher)
  static int64_t steadyNowNs();

  // Core logic; core_mutex_ serializes inference (worker) and calibration updates (callbacks)
  std::unique_ptr<BEVFusionCore> core_;
  BEVFusionInputConfig config_;
  std::mutex core_mutex_;

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Frame IDs
  std::string lidar_frame_id_;

  // Cameras
  std::vector<std::string> camera_names_;
  CameraInput camera_input_{CameraInput::kCompressed};
  IntrinsicsSource intrinsics_source_{IntrinsicsSource::kProjection};

  // Camera info, one per camera in camera_names_ order
  std::vector<rclcpp::Subscription<CameraInfoMsg>::SharedPtr> camera_info_subs_;
  std::vector<CameraInfoMsg::ConstSharedPtr> cached_camera_infos_;
  std::vector<double> camera_azimuths_deg_;  // viewing direction of each camera in the LiDAR frame
  std::mutex camera_info_mutex_;

  // Calibration
  std::atomic<bool> calibration_initialized_{false};

  // Input topics
  std::string lidar_topic_;

  // 3D detection: TF target frame
  std::string target_frame_;

  // Ego-vehicle suppression radius
  float ego_suppression_radius_{0.0f};

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<vision_msgs::msg::Detection3DArray>::SharedPtr detection_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Subscribers (only the ones for camera_input_ are created)
  rclcpp::Subscription<PointCloud2Msg>::SharedPtr lidar_sub_;
  std::vector<rclcpp::Subscription<CompressedImageMsg>::SharedPtr> compressed_image_subs_;
  std::vector<std::shared_ptr<NitrosImageSub>> nitros_image_subs_;

  // QoS profiles
  rclcpp::QoS subscriber_qos_;
  rclcpp::QoS publisher_qos_;

  // Camera matching (see camera_matcher.hpp); matcher_mutex_ guards the matcher and its deadline timer
  CameraMatcherConfig matcher_config_;
  SyncTarget sync_target_{SyncTarget::kSweep};
  double lidar_rotation_period_ms_{50.0};
  std::unique_ptr<Matcher> matcher_;
  rclcpp::TimerBase::SharedPtr deadline_timer_;
  int64_t armed_deadline_ns_{-1};
  // A matcher reset (backward time jump) starts a new epoch: sets are ordered by (epoch, scan stamp)
  uint64_t matcher_epoch_{0};
  uint64_t matcher_resets_seen_{0};
  // Steady-clock arrival of each camera's newest frame (-1: none yet) and when matching (re)started
  std::vector<int64_t> last_frame_arrival_ns_;
  int64_t matching_started_ns_{0};
  // Camera intrinsics (N x 4 x 4) last sent to the core, for diagnostics and tests; camera_info_mutex_
  std::vector<float> calibration_intrinsics_;
  std::mutex matcher_mutex_;

  // Inference worker: a single-slot mailbox, so a newer set replaces one the worker has not started
  std::thread worker_;
  std::mutex mailbox_mutex_;
  std::condition_variable mailbox_cv_;
  std::optional<MatchedSet> mailbox_;
  uint64_t mailbox_epoch_{0};
  bool stop_worker_{false};

  // Statistics
  std::atomic<uint64_t> total_processed_{0};
  std::atomic<double> total_processing_time_ms_{0.0};
  std::atomic<double> last_processing_time_ms_{0.0};
  std::chrono::steady_clock::time_point last_stats_log_time_;
  static constexpr std::chrono::seconds kStatsLogInterval{30};

  // Statistics
  std::atomic<uint64_t> image_msg_count_{0};
  std::atomic<uint64_t> lidar_msg_count_{0};
  std::atomic<uint64_t> matched_set_count_{0};
  std::atomic<uint64_t> superseded_set_count_{0};
  std::atomic<uint64_t> sweep_fallback_count_{0};
  std::atomic<double> last_max_gap_ms_{0.0};

  // Diagnostics
  std::unique_ptr<diagnostic_updater::Updater> diagnostic_updater_;
  std::unique_ptr<diagnostic_updater::TopicDiagnostic> pub_diagnostic_;
  double min_freq_{0.0};
  double max_freq_{100.0};
};

}  // namespace wato::perception::bevfusion
