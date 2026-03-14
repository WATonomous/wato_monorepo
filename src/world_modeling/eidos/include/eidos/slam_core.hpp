#pragma once

#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_lifecycle/lifecycle_publisher.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

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
 * Uses a single timeline model: each plugin independently creates states
 * at its own cadence via hasData()/getFactors(). A motion model plugin
 * connects every consecutive state pair as a backbone, guaranteeing
 * full-rank constraints and kinematically feasible trajectories.
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
  uint64_t getCurrentStateIndex() const;
  const gtsam::NonlinearFactorGraph& getAccumulatedGraph() const;
  const std::vector<std::string>& getAccumulatedFactorOwners() const;
  std::optional<gtsam::Pose3> getMotionModelPose() const;

  /**
   * @brief Get the ISAM2 marginal covariance of the latest optimized pose.
   * Returns 6x6 matrix [rot3 | pos3]. Returns nullopt if no state exists yet.
   */
  std::optional<Eigen::MatrixXd> getLatestPoseCovariance() const;

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
  void visLoop();
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
  void publishPose();
  void publishOdometry();

  // ---- TF ----
  void broadcastMapToOdom();

  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string odom_frame_;
  bool publish_map_to_odom_ = false;
  gtsam::Pose3 cached_map_to_odom_;  // identity until first optimization with valid odom->base TF

  // ---- SLAM state ----
  SlamState state_ = SlamState::INITIALIZING;
  gtsam::Pose3 current_pose_;
  float current_transform_[6] = {0};

  // Motion model delta tracking: current_pose = last_optimized * delta(mm)
  gtsam::Pose3 last_optimized_pose_;
  gtsam::Pose3 mm_pose_at_last_optimization_;
  bool has_optimization_anchor_ = false;

  // ---- Single-timeline state tracking ----
  uint64_t next_state_index_ = 0;   // monotonic counter for Symbol('x', N)
  gtsam::Key last_state_key_ = 0;
  double last_state_timestamp_ = 0.0;
  std::string last_state_owner_;     // plugin name that created the last state ("" for state 0)
  bool has_last_state_ = false;

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

  // ---- Timers ----
  rclcpp::TimerBase::SharedPtr slam_timer_;
  rclcpp::CallbackGroup::SharedPtr slam_callback_group_;
  rclcpp::TimerBase::SharedPtr vis_timer_;
  rclcpp::CallbackGroup::SharedPtr vis_callback_group_;

  // ---- Visualization state (written by SLAM loop, read by vis timer) ----
  gtsam::Values vis_values_;
  bool vis_loop_closure_ = false;
  bool vis_pending_ = false;
  std::mutex vis_mtx_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<eidos_msgs::msg::SlamStatus>::SharedPtr status_pub_;
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  // ---- Services ----
  rclcpp::Service<eidos_msgs::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::Service<eidos_msgs::srv::LoadMap>::SharedPtr load_map_srv_;

  // ---- Parameters (populated from ROS params in onConfigure) ----
  double slam_rate_;
  double relocalization_timeout_;
  std::string map_frame_;
  std::string base_link_frame_;

  // Map parameters
  std::string map_load_directory_;
  std::string map_save_directory_;

  // Prior noise: diagonal of 6D Pose3 covariance [roll, pitch, yaw, x, y, z]
  std::vector<double> prior_pose_cov_ = {1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-2};

  // Published odometry covariance diagonal [x, y, z, roll, pitch, yaw]
  std::vector<double> odom_pose_cov_ = {1.0, 1.0, 1.0, 0.1, 0.1, 0.1};

  // ISAM2 parameters
  int isam2_update_iterations_ = 2;        // iterations per normal update
  int isam2_correction_iterations_ = 5;    // extra iterations on GPS/loop closure
  double isam2_relinearize_threshold_ = 0.1;
  int isam2_relinearize_skip_ = 1;

  // ---- Loop closure tracking ----
  bool loop_closure_detected_ = false;

  // Accumulated factor graph for serialization
  gtsam::NonlinearFactorGraph accumulated_graph_;
  gtsam::Values accumulated_values_;
  std::vector<std::string> accumulated_factor_owners_;  // parallel to accumulated_graph_

  // ---- Relocalization timing ----
  double relocalization_start_time_ = 0.0;

  std::mutex mtx_;
};

}  // namespace eidos
