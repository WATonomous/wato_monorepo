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
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>

#include <pluginlib/class_loader.hpp>

#include "eidos/types.hpp"
#include "eidos/pose_graph.hpp"
#include "eidos/map_manager.hpp"
#include "eidos/plugins/base_factor_plugin.hpp"
#include "eidos/plugins/base_relocalization_plugin.hpp"
#include "eidos/plugins/base_visualization_plugin.hpp"

#include "eidos_msgs/msg/slam_status.hpp"
#include "eidos_msgs/srv/save_map.hpp"
#include "eidos_msgs/srv/load_map.hpp"

namespace eidos {

/**
 * @brief Lifecycle-managed SLAM node. Runs a timer-driven SLAM loop,
 * manages plugins, PoseGraph, and MapManager.
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
  int getCurrentStateIndex() const;

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
  void handleRelocalizing(double timestamp);
  void handleTracking(double timestamp, bool& run_vis,
                      gtsam::Values& vis_values, bool& vis_loop_closure);

  // ---- Plugin management ----
  void loadFactorPlugins();
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
  void updatePath(const PoseType& pose);
  void broadcastMapToOdom();

  // ---- TF ----
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // ---- SLAM state ----
  SlamState state_ = SlamState::INITIALIZING;
  gtsam::Pose3 current_pose_;
  int current_state_index_ = -1;
  float current_transform_[6] = {0};

  // ---- Core components ----
  std::unique_ptr<PoseGraph> pose_graph_;
  std::unique_ptr<MapManager> map_manager_;

  // ---- Plugins ----
  std::unique_ptr<pluginlib::ClassLoader<FactorPlugin>> factor_plugin_loader_;
  std::unique_ptr<pluginlib::ClassLoader<RelocalizationPlugin>> reloc_plugin_loader_;
  std::unique_ptr<pluginlib::ClassLoader<VisualizationPlugin>> vis_plugin_loader_;
  std::vector<std::shared_ptr<FactorPlugin>> factor_plugins_;
  std::vector<std::shared_ptr<RelocalizationPlugin>> reloc_plugins_;
  std::vector<std::shared_ptr<VisualizationPlugin>> vis_plugins_;

  // ---- Timer ----
  rclcpp::TimerBase::SharedPtr slam_timer_;
  rclcpp::CallbackGroup::SharedPtr slam_callback_group_;

  // ---- Publishers ----
  rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp_lifecycle::LifecyclePublisher<eidos_msgs::msg::SlamStatus>::SharedPtr status_pub_;

  // ---- Services ----
  rclcpp::Service<eidos_msgs::srv::SaveMap>::SharedPtr save_map_srv_;
  rclcpp::Service<eidos_msgs::srv::LoadMap>::SharedPtr load_map_srv_;

  // ---- Path ----
  nav_msgs::msg::Path global_path_;

  // ---- Parameters ----
  double slam_rate_ = 10.0;
  double max_displacement_ = 1.0;
  double max_rotation_ = 0.2;
  double relocalization_timeout_ = 30.0;
  std::string map_frame_ = "map";
  std::string odom_frame_ = "odom";
  std::string base_link_frame_ = "base_link";

  // Keyframe parameters
  float keyframe_density_ = 2.0;
  float keyframe_search_radius_ = 50.0;

  // Map parameters
  std::string map_load_directory_;
  std::string map_save_directory_;

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
