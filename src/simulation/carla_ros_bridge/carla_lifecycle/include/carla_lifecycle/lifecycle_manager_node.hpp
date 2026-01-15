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

#ifndef CARLA_LIFECYCLE__LIFECYCLE_MANAGER_NODE_HPP_
#define CARLA_LIFECYCLE__LIFECYCLE_MANAGER_NODE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "carla_msgs/msg/scenario_status.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "rclcpp/rclcpp.hpp"

namespace carla_lifecycle
{

struct NodeClients
{
  rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr change_state;
  rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr get_state;
};

struct TransitionStep
{
  uint8_t transition_id;
  uint8_t required_state;
  const char * name;
};

class LifecycleManagerNode : public rclcpp::Node
{
public:
  explicit LifecycleManagerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void startupTimerCallback();
  void scenarioStatusCallback(const carla_msgs::msg::ScenarioStatus::SharedPtr msg);

  bool bringUpNode(const std::string & node_name);
  void bringUpAllNodes();
  void restartManagedNodes();
  void executeTransitionSteps(const std::vector<TransitionStep> & steps);

  bool changeState(const std::string & node_name, uint8_t transition_id);
  int getNodeState(const std::string & node_name);
  std::string stateName(int state_id);

  // Parameters
  bool autostart_;
  std::string scenario_server_name_;
  std::vector<std::string> node_names_;
  double service_timeout_;

  // State tracking
  std::string current_scenario_;
  std::string last_scenario_state_;
  bool startup_complete_ = false;

  // Service clients - initialized once at startup
  std::map<std::string, NodeClients> clients_;

  // ROS interfaces
  rclcpp::Subscription<carla_msgs::msg::ScenarioStatus>::SharedPtr scenario_sub_;
  rclcpp::TimerBase::SharedPtr startup_timer_;
  rclcpp::CallbackGroup::SharedPtr service_cb_group_;
};

}  // namespace carla_lifecycle

#endif  // CARLA_LIFECYCLE__LIFECYCLE_MANAGER_NODE_HPP_
