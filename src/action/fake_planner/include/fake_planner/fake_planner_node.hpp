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

#include <memory>
#include <string>

#include "behaviour_msgs/msg/execute_behaviour.hpp"
#include "fake_planner/fake_planner_core.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace fake_planner
{

// Replays a predefined trajectory (expanded from a maneuver JSON segment list) at a fixed rate so a
// controller can be exercised without the real planner stack, plus an execute_behaviour heartbeat
// so the controller leaves standby. The same node works in sim and on the vehicle; only the
// odom/command topics differ, set via remaps in the launch file.
class FakePlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  explicit FakePlannerNode(const rclcpp::NodeOptions & options);

  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  // Publishes the current window on `trajectory_markers`, only when someone is subscribed.
  void publishMarkers();
  void timerCallback();
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void startTrajectory(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void stopTrajectory(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  // Re-runs the maneuver from wherever the car currently sits, without a relaunch.
  void resetTrajectory(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  std::unique_ptr<FakePlannerCore> core_;

  // Parameters
  std::string trajectory_topic_;
  std::string behaviour_topic_;
  std::string odom_topic_;
  std::string frame_id_;
  double publish_rate_hz_{10.0};
  bool publish_behaviour_{true};
  std::string behaviour_{"lane_follow"};
  // "auto" | "relative" | "absolute". Under "auto" the file decides: a maneuver carrying an
  // absolute "start" is published verbatim, anything else is laid out from the vehicle's pose.
  std::string anchoring_{"auto"};
  bool anchor_to_first_pose_{true};
  bool start_on_activate_{true};
  std::string maneuver_file_;
  std::string marker_topic_;
  double horizon_m_{35.0};
  double trail_m_{2.0};
  double respawn_jump_m_{5.0};
  double finish_distance_m_{3.0};
  double finish_speed_mps_{0.25};

  // Gates publishing: when false the node stays silent so the controller holds in standby.
  // Toggled by the start/stop services; seeded from start_on_activate on activation.
  bool trajectory_started_{true};

  // Latched when the vehicle reaches the end of an open maneuver, so the controller times out into
  // standby and the car stays put rather than creeping or silently starting over. Cleared only by
  // an explicit reset (or a respawn, which is the sim's reset).
  bool finished_{false};

  bool anchored_{false};

  // Latest vehicle position in frame_id_, used to slide the window, and its speed magnitude, which
  // decides whether it has actually come to rest at the end of the maneuver.
  bool have_pose_{false};
  double veh_x_{0.0};
  double veh_y_{0.0};
  double veh_speed_{0.0};

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp_lifecycle::LifecyclePublisher<wato_trajectory_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp_lifecycle::LifecyclePublisher<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr behaviour_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace fake_planner
