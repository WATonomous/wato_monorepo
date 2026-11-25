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
#include <mutex>
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

/**
 * @brief Lifecycle node that loads and replays a maneuver for controller testing.
 *
 * Configure loads the JSON and creates ROS interfaces. Activate starts publishing.
 * Start/stop services control release; reset prepares the loaded maneuver again.
 * Topic routing belongs in the launch files.
 */
class FakePlannerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

  /**
   * @brief Declare parameters for the fake planner.
   * @param options ROS node options and parameter overrides.
   */
  explicit FakePlannerNode(const rclcpp::NodeOptions & options);

  /**
   * @brief Load the trajectory and create publishers, subscribers and services.
   * @param state Previous lifecycle state.
   * @return Success when the configuration and maneuver are valid.
   */
  CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Activate publishers and start the publication timer.
   * @param state Previous lifecycle state.
   * @return Success after activation.
   */
  CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Stop the timer and deactivate publishers.
   * @param state Previous lifecycle state.
   * @return Success after deactivation.
   */
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Release the loaded trajectory and ROS interfaces.
   * @param state Previous lifecycle state.
   * @return Success after cleanup.
   */
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

  /**
   * @brief Clean up before shutdown.
   * @param state Previous lifecycle state.
   * @return Result of cleanup.
   */
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

private:
  /** @brief Validate parameters and load the JSON. @return True when ready to configure. */
  bool loadTrajectory();

  /** @brief Reset matching, anchoring and completion state for the loaded maneuver. */
  void resetTrajectory();

  /** @brief Publish the current window and behavior heartbeat while released. */
  void publishTrajectory();

  /** @brief Release the trajectory and start its speed ramp. */
  void releaseTrajectory();

  /** @brief Calculate release ramp progress. @return Speed multiplier between zero and one. */
  double releaseRampScale();

  /**
   * @brief Publish the trajectory preview and speed labels.
   * @param trajectory Window with the speeds currently being published.
   */
  void publishMarkers(const wato_trajectory_msgs::msg::Trajectory & trajectory);

  /**
   * @brief Anchor the maneuver and update the window from odometry.
   * @param msg Vehicle pose and velocity in the trajectory frame.
   */
  void odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr & msg);

  /**
   * @brief Handle the start service.
   * @param request Unused Trigger request.
   * @param response Reports whether publication was started.
   */
  void startTrajectoryCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * @brief Handle the stop service.
   * @param request Unused Trigger request.
   * @param response Reports that the controller will return to standby.
   */
  void stopTrajectoryCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  /**
   * @brief Handle the reset service without reloading the JSON file.
   * @param request Unused Trigger request.
   * @param response Reports whether the loaded maneuver was reset.
   */
  void resetTrajectoryCallback(
    const std_srvs::srv::Trigger::Request::SharedPtr request, std_srvs::srv::Trigger::Response::SharedPtr response);

  // Trajectory helper
  std::unique_ptr<FakePlannerCore> core_;
  std::mutex core_mutex_;

  // Parameters
  double publish_rate_hz_{10.0};
  bool publish_behaviour_{true};
  std::string behaviour_{"lane_follow"};
  bool anchor_to_first_pose_{true};
  bool start_on_activate_{true};
  double respawn_jump_m_{5.0};
  double finish_distance_m_{3.0};
  double finish_speed_mps_{0.25};
  double max_path_deviation_m_{5.0};
  double release_ramp_s_{3.0};

  // Playback state
  bool trajectory_started_{false};
  bool finished_{false};
  bool moved_{false};
  rclcpp::Time release_time_;
  bool anchored_{false};
  bool have_pose_{false};
  double veh_x_{0.0};
  double veh_y_{0.0};
  double veh_speed_{0.0};

  // Publishers
  rclcpp_lifecycle::LifecyclePublisher<wato_trajectory_msgs::msg::Trajectory>::SharedPtr trajectory_pub_;
  rclcpp_lifecycle::LifecyclePublisher<behaviour_msgs::msg::ExecuteBehaviour>::SharedPtr behaviour_pub_;
  rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  // Subscriber, timer and services
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_srv_;
};

}  // namespace fake_planner
