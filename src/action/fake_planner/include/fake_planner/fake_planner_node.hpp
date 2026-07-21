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
#include <vector>

#include "behaviour_msgs/msg/execute_behaviour.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "wato_trajectory_msgs/msg/trajectory.hpp"

namespace fake_planner
{

// Replays a predefined trajectory (expanded from a maneuver JSON segment list) at a fixed rate so a
// controller can be exercised without the real planner stack. Also publishes an
// execute_behaviour heartbeat so the controller leaves standby. Environment-agnostic: the
// same node works in the CARLA sim and on the vehicle — only the odom/command topics differ
// (set via remaps in the launch file).
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
  // Parses a maneuver JSON (segment list), expands the segments into a smooth path and resamples
  // it to uniform spacing in wp_x_/wp_y_/wp_speed_. Returns false and sets `error` on any I/O,
  // JSON, or schema failure.
  bool loadManeuver(const std::string & path, std::string & error);
  // Builds full_traj_ from the loaded waypoints, applying the current anchor transform.
  void buildTrajectory();
  // Slices the rolling window out of full_traj_ around the vehicle's current position into
  // trajectory_, mirroring what the real planner emits (a short horizon, not the whole route).
  void updateWindow();
  // Index of the full-path point closest to (veh_x_, veh_y_), searched forward from the last
  // match so a self-intersecting maneuver doesn't snap back to an earlier lap.
  std::size_t nearestIndex() const;
  // Publishes the trajectory as a MarkerArray, mirroring trajectory_planner's visualization
  // (speed-sized spheres + speed labels on `trajectory_markers`, only when subscribed).
  void publishMarkers();
  void timerCallback();
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);
  void startTrajectory(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  void stopTrajectory(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);
  // Re-runs the maneuver from wherever the car currently sits, without a relaunch: rewinds the
  // window and (when anchoring) re-lays the path from the next odom pose.
  void resetTrajectory(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  // Parameters
  std::string trajectory_topic_;
  std::string behaviour_topic_;
  std::string odom_topic_;
  std::string frame_id_;
  double publish_rate_hz_{10.0};
  bool publish_behaviour_{true};
  std::string behaviour_{"lane_follow"};
  bool anchor_to_first_pose_{true};
  bool start_on_activate_{true};
  std::string maneuver_file_;
  std::string marker_topic_;
  double horizon_m_{30.0};
  double trail_m_{2.0};
  double respawn_jump_m_{5.0};

  // Set from the maneuver's "closed" flag: a closed circuit laps forever (the window wraps
  // past the end), an open maneuver ends in a stop.
  bool closed_{false};

  // Waypoints expanded from the maneuver segments (parallel arrays; relative to the anchor pose).
  std::vector<double> wp_x_;
  std::vector<double> wp_y_;
  std::vector<double> wp_speed_;

  // The whole anchored maneuver (built once the anchor pose is known), and the rolling window
  // of it that actually gets published each tick.
  wato_trajectory_msgs::msg::Trajectory full_traj_;
  wato_trajectory_msgs::msg::Trajectory trajectory_;
  bool trajectory_ready_{false};
  std::size_t window_start_{0};

  // Gates publishing: when false the node stays silent so the controller holds in standby.
  // Toggled by the start/stop services; seeded from start_on_activate on activation.
  bool trajectory_started_{true};

  // Anchor pose (SE(2), in frame_id_) that the waypoints are laid out from.
  bool anchored_{false};
  double anchor_x_{0.0};
  double anchor_y_{0.0};
  double anchor_yaw_{0.0};

  // Latest vehicle position in frame_id_, tracked continuously to slide the window.
  bool have_pose_{false};
  double veh_x_{0.0};
  double veh_y_{0.0};

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
