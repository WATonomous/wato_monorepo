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

#ifndef ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
#define ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_

#include <memory>
#include <string>

#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "ackermann_pure_pursuit/pure_pursuit_math.hpp"
#include "behaviour_msgs/msg/execute_behaviour.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/timer.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/bool.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "wato_trajectory_msgs/msg/controller_status.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace ackermann_pure_pursuit
{

class PurePursuitNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  explicit PurePursuitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  void controlCallback();
  void trajectoryCallback(const wato_trajectory_msgs::msg::Trajectory::SharedPtr msg);
  void bt_callback(const behaviour_msgs::msg::ExecuteBehaviour::ConstSharedPtr & msg);
  double getWheelbase();
  void publishAckermannMsg(std::string frame_id, double speed, double steering_angle, bool invert_steering = false);
  void publishStandby();
  void loadParameters();

  // Transform the trajectory into `rear_axle_frame_`. Returns false if the TF is unavailable.
  bool buildPathInReferenceFrame(
    const wato_trajectory_msgs::msg::Trajectory & traj,
    std::vector<math::Vec2> & path_out,
    std::vector<double> & speeds_out);

  // Advance the debounced disengage state machine. `over` = a threshold is exceeded
  // this cycle; returns the current disengaged state.
  bool updateDisengage(bool over, const std::string & reason, const rclcpp::Time & t_now);

  // Parameters — topics/frames
  std::string trajectory_topic_;
  std::string bt_topic_;
  std::string ackermann_topic_;
  std::string idle_topic_;
  std::string controller_status_topic_;
  std::string base_frame_;
  std::string rear_axle_frame_;
  std::string front_axle_frame_;
  std::string standby_msg_;

  // Parameters — lookahead (Phase 2)
  double lookahead_distance_;        // max lookahead (m)
  double min_lookahead_distance_;    // min lookahead (m)
  double lookahead_time_;            // time-headway: ld = lookahead_time * speed (s)
  double curvature_lookahead_gain_;  // shrink factor on curvature: ld_eff = ld / (1 + gain*|kappa|)
  double speed_lookahead_distance_;  // minimum distance ahead used to sample target speed (m)
  double speed_lookahead_time_;      // time-headway for the speed sample horizon (s)

  // Parameters — steering
  double steering_angle_gain_;
  double max_steering_angle_;
  double max_steering_rate_;  // rad/s

  // Parameters — speed (Phase 3)
  double max_speed_;
  double min_speed_;
  double max_lateral_accel_;  // m/s^2, curvature-limited speed budget
  double max_accel_;          // m/s^2 command slew up
  double max_decel_;          // m/s^2 command slew down

  // Parameters — Stanley low-speed blend (Phase 3d)
  bool enable_stanley_blend_;
  double stanley_gain_;
  double stanley_softening_;
  double stanley_speed_threshold_;

  // Parameters — disengage monitor (Phase 4)
  bool enable_disengage_;
  double max_cross_track_error_;
  double max_heading_error_;
  double disengage_debounce_sec_;
  bool disengage_latch_;
  double disengage_speed_;

  // Parameters — standby / misc
  double standby_speed_;
  double standby_steering_;
  double control_rate_hz_;
  double wheelbase_fallback_;
  double idle_timeout_sec_;
  bool invert_steering_;
  bool disable_standby_;

  // Cached wheelbase from TF (0.0 = not yet resolved)
  double wheelbase_cached_{0.0};

  // TF
  std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

  // Pub/Sub
  rclcpp_lifecycle::LifecyclePublisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr ackermann_pub_;
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Bool>::SharedPtr idle_pub_;
  rclcpp_lifecycle::LifecyclePublisher<wato_trajectory_msgs::msg::ControllerStatus>::SharedPtr status_pub_;
  rclcpp::Subscription<wato_trajectory_msgs::msg::Trajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr bt_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // State
  wato_trajectory_msgs::msg::Trajectory::SharedPtr latest_trajectory_;
  rclcpp::Time last_trajectory_time_;
  std::string bt_requested_behaviour_;
  double current_speed_{0.0};

  // Command-shaping state (previous cycle outputs for slew limiting)
  double prev_speed_cmd_{0.0};
  double prev_steering_cmd_{0.0};
  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};
  bool have_last_control_{false};

  // Disengage debounce state
  rclcpp::Time over_threshold_since_{0, 0, RCL_ROS_TIME};
  bool over_threshold_active_{false};
  bool disengaged_latched_{false};
  std::string disengage_reason_;
};

}  // namespace ackermann_pure_pursuit

#endif  // ACKERMANN_PURE_PURSUIT__PURE_PURSUIT_NODE_HPP_
