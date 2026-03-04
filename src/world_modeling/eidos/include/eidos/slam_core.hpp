#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pluginlib/class_loader.hpp>
#include <gtsam/inference/Symbol.h>

#include "eidos/types.hpp"
#include "eidos/pose_graph.hpp"
#include "eidos/map_manager.hpp"
#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/plugins/base_motion_model_plugin.hpp"
#include "eidos/plugins/base_relocalization_plugin.hpp"
#include "eidos/plugins/base_visualization_plugin.hpp"

#include "eidos_msgs/msg/slam_status.hpp"
#include "eidos_msgs/srv/save_map.hpp"
#include "eidos_msgs/srv/load_map.hpp"

namespace eidos {

/**
 * @brief Lifecycle-managed SLAM node. Runs a timer-driven SLAM loop,
 * manages plugins, PoseGraph, and MapManager.
 *
 * Uses a keygroup model: each SLAM tick (when displacement is met) forms
 * a keygroup. Each factor plugin contributes 0 or 1 states at its own
 * sensor timestamp. A motion model plugin generates constraints between
 * consecutive states across keygroups.
 */
class SlamCore : public rclcpp_lifecycle::LifecycleNode {
public:
  explicit SlamCore(const rclcpp::NodeOptions& options);
  ~SlamCore() override;

  // ---- Access for plugins ----
  const PoseGraph& getPoseGraph() const;
  const MapManager& getMapManager() const;
  MapManager& getMapManager();
  SlamState getState() const;
  gtsam::Pose3 getCurrentPose() const;
  int getCurrentKeygroup() const;

protected:
  using CallbackReturn =
      rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

private:
  // ---- SLAM loop ----
  void slamLoop();
  void handleInitializing();
  void handleWarmingUp(double timestamp);
  void handleRelocalizing(double timestamp);
  void handleTracking(double timestamp, bool& run_vis,
                      gtsam::Values& vis_values, bool& vis_loop_closure);
  void beginTracking(const gtsam::Pose3& initial_pose, double timestamp);

  // ---- Plugin management ----
  void loadFactorPlugins();
  void loadMotionModel();
  void loadRelocalizationPlugins();
  void loadVisualizationPlugins();

  // ---- Services ----
  void saveMapCallback(
      const std::shared_ptr<eidos_msgs::srv::SaveMap::Request> request,
      std::shared_ptr<eidos_msgs::srv::SaveMap::Response> response);
  void loadMapCallback(
      const std::shared_ptr<eidos_msgs::srv::LoadMap::Request> request,
      std::shared_ptr<eidos_msgs::srv::LoadMap::Response> response);

  // ---- Publishing ----
  void publishStatus();
  void publishPath();
  void publishPose();
  void publishOdometry();
  void updatePath(const PoseType& pose);

  // ---- TF ----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // ---- SLAM state ----
  SlamState state_ = SlamState::INITIALIZING;
  gtsam::Pose3 current_pose_;
  int current_keygroup_ = -1;
  float current_transform_[6] = {0};

  // ---- Keygroup state tracking ----
  struct KeygroupState {
    gtsam::Key key;
    double timestamp;
  };
  KeygroupState last_keygroup_state_;  // last state (by timestamp) from previous keygroup

  // ---- Core components ----
  std::unique_ptr<PoseGraph> pose_graph_;
  std::unique_ptr<MapManager> map_manager_;

  // ---- Plugins ----
  std::unique_ptr<pluginlib::ClassLoader<FactorPlugin>> factor_plugin_loader_;
  std::unique_ptr<pluginlib::ClassLoader<MotionModelPlugin>> motion_model_loader_;
  std::unique_ptr<pluginlib::ClassLoader<RelocalizationPlugin>> reloc_plugin_loader_;
  std::unique_ptr<pluginlib::ClassLoader<VisualizationPlugin>> vis_plugin_loader_;
  std::vector<std::shared_ptr<FactorPlugin>> factor_plugins_;
  std::shared_ptr<MotionModelPlugin> motion_model_;
  std::vector<std::shared_ptr<RelocalizationPlugin>> reloc_plugins_;
  std::vector<std::shared_ptr<VisualizationPlugin>> vis_plugins_;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr slam_timer_;
  rclcpp::CallbackGroup::SharedPtr slam_callback_group_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<eidos_msgs::msg::SlamStatus>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // ---- Services ----
  rclcpp::Service<eidos_msgs::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::Service<eidos_msgs::srv::LoadMap>::SharedPtr load_map_srv_;

  // ---- Path ----
  nav_msgs::msg::Path global_path_;

  // ---- Parameters (populated from ROS params in onConfigure) ----
  double slam_rate_;
  double max_displacement_;
  double max_rotation_;
  double relocalization_timeout_;
  std::string map_frame_;
  std::string odom_frame_;
  std::string base_link_frame_;

  // Keyframe parameters
  float keyframe_density_;
  float keyframe_search_radius_;

  // Map parameters
  std::string map_load_directory_;
  std::string map_save_directory_;

  // Prior noise: diagonal of 6D Pose3 covariance [roll, pitch, yaw, x, y, z]
  std::vector<double> prior_pose_cov_ = {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

  // Published odometry covariance diagonal [x, y, z, roll, pitch, yaw]
  std::vector<double> odom_pose_cov_ = {1.0, 1.0, 1.0, 0.1, 0.1, 0.1};

  // ---- Loop closure tracking ----
  bool loop_closure_detected_ = false;

  // Accumulated factor graph for serialization
  gtsam::NonlinearFactorGraph accumulated_graph_;
  gtsam::Values accumulated_values_;

  // ---- Relocalization timing ----
  double relocalization_start_time_ = 0.0;

  std::mutex mtx_;
};

}  // namespace eidos
