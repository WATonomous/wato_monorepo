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

#include <behaviortree_cpp_v3/action_node.h>
#include <behaviortree_cpp_v3/async_action_node.h>
#include <behaviortree_cpp_v3/bt_factory.h>

#include "behaviour/bt_node_utils.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "world_modeling_msgs/srv/behaviour_tree_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include <mutex>
#include <vector>

using BehaviourTreeInfo = world_modeling_msgs::srv::BehaviourTreeInfo;

namespace wato::world_modeling::behaviour
{

class GetHDMapInfo : public BT::SyncActionNode
{
public:
  GetHDMapInfo(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = get_shared_node();
    if (!node_) {
      throw std::runtime_error("GetHDMapInfo: shared node not set");
    }
    client_ = node_->create_client<BehaviourTreeInfo>("behaviour_tree_info");
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<geometry_msgs::msg::Point>("current_point"),
             BT::OutputPort<geometry_msgs::msg::Point>("goal_point") };
  }

  BT::NodeStatus tick() override
  {
    if (!client_->wait_for_service(std::chrono::milliseconds(500))) {
      RCLCPP_WARN(node_->get_logger(), "GetHDMapInfo: behaviour_tree_info service not available");
      return BT::NodeStatus::FAILURE;
    }
    auto req = std::make_shared<BehaviourTreeInfo::Request>();
    auto fut = client_->async_send_request(req);
    if (rclcpp::spin_until_future_complete(node_, fut, std::chrono::seconds(1))
        != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_WARN(node_->get_logger(), "GetHDMapInfo: service call failed/timeout");
      return BT::NodeStatus::FAILURE;
    }
    auto res = fut.get();
    setOutput("current_point", res->current_point);
    setOutput("goal_point", res->goal_point);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Client<BehaviourTreeInfo>::SharedPtr client_;
};


class RequestRoute : public BT::SyncActionNode
{
public:
  RequestRoute(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = get_shared_node();
    if (!node_) throw std::runtime_error("RequestRoute: shared node not set");
    std::string topic = "clicked_point";
    // declare/get param so user can override via node params
    node_->declare_parameter<std::string>("hd_map_clicked_point_topic", topic);
    node_->get_parameter("hd_map_clicked_point_topic", topic);
    pub_ = node_->create_publisher<geometry_msgs::msg::PointStamped>(topic, 1);
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<geometry_msgs::msg::Point>("goal_point") };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Point goal;
    if (!getInput("goal_point", goal)) {
      RCLCPP_WARN(node_->get_logger(), "RequestRoute: no goal_point in input ports");
      return BT::NodeStatus::FAILURE;
    }
    geometry_msgs::msg::PointStamped ps;
    ps.header.stamp = node_->now();
    ps.header.frame_id = "map";
    ps.point = goal;
    pub_->publish(ps);
    RCLCPP_INFO(node_->get_logger(), "RequestRoute: published clicked_point (%.2f, %.2f)", goal.x, goal.y);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_;
};


class WaitForRoute : public BT::AsyncActionNode
{
public:
  WaitForRoute(const std::string & name, const BT::NodeConfiguration & config)
  : BT::AsyncActionNode(name, config), received_(false)
  {
    node_ = get_shared_node();
    if (!node_) throw std::runtime_error("WaitForRoute: shared node not set");
    std::string topic = "hd_map_route";
    node_->declare_parameter<std::string>("hd_map_route_topic", topic);
    node_->get_parameter("hd_map_route_topic", topic);
    sub_ = node_->create_subscription<visualization_msgs::msg::MarkerArray>(
      topic, 1, [this](const visualization_msgs::msg::MarkerArray::SharedPtr msg) {
        std::lock_guard<std::mutex> lk(mutex_);
        last_msg_ = msg;
        received_ = true;
      });
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<int>("timeout_ms", 2000) };
  }

  BT::NodeStatus tick() override
  {
    int timeout_ms = 2000;
    getInput("timeout_ms", timeout_ms);
    const auto start = node_->now();
    rclcpp::Rate r(10);
    while (rclcpp::ok()) {
      {
        std::lock_guard<std::mutex> lk(mutex_);
        if (received_) {
          // clear flag so next WaitForRoute will wait for a new message
          received_ = false;
          RCLCPP_INFO(node_->get_logger(), "WaitForRoute: route received");
          return BT::NodeStatus::SUCCESS;
        }
      }
      if ((node_->now() - start).seconds() * 1000.0 > timeout_ms) {
        RCLCPP_WARN(node_->get_logger(), "WaitForRoute: timeout waiting for hd_map_route");
        return BT::NodeStatus::FAILURE;
      }
      r.sleep();
    }
    return BT::NodeStatus::FAILURE;
  }

  void halt() override
  {
    // nothing special
    setStatus(BT::NodeStatus::IDLE);
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr sub_;
  std::mutex mutex_;
  visualization_msgs::msg::MarkerArray::SharedPtr last_msg_;
  bool received_;
};


// --- Stub nodes below ---

class MonitorTrafficLights : public BT::SyncActionNode
{
public:
  MonitorTrafficLights(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = get_shared_node();
  }

  static BT::PortsList providedPorts()
  {
    return { BT::OutputPort<int>("state") };
  }

  BT::NodeStatus tick() override
  {
    // Stub: always report "go" state (1).
    if (node_) {
      RCLCPP_DEBUG(node_->get_logger(), "MonitorTrafficLights (stub): setting state=1");
    }
    setOutput("state", 1);
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

class CheckHaveValidTrajectory : public BT::SyncActionNode
{
public:
  CheckHaveValidTrajectory(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = get_shared_node();
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<bool>("have_valid") };
  }

  BT::NodeStatus tick() override
  {
    bool have_valid = false;
    getInput("have_valid", have_valid);
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "CheckHaveValidTrajectory (stub): have_valid=%s", have_valid ? "true" : "false");
    }
    return have_valid ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

class RunPlanner : public BT::SyncActionNode
{
public:
  RunPlanner(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = get_shared_node();
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<geometry_msgs::msg::Point>("start"),
      BT::InputPort<geometry_msgs::msg::Point>("goal"),
      BT::OutputPort<std::vector<geometry_msgs::msg::Point>>("planned_path")
    };
  }

  BT::NodeStatus tick() override
  {
    geometry_msgs::msg::Point start;
    geometry_msgs::msg::Point goal;
    if (!getInput("start", start) || !getInput("goal", goal)) {
      if (node_) {
        RCLCPP_WARN(node_->get_logger(), "RunPlanner (stub): missing start/goal");
      }
      return BT::NodeStatus::FAILURE;
    }

    // Stub path: straight line start -> goal.
    std::vector<geometry_msgs::msg::Point> path;
    path.push_back(start);
    path.push_back(goal);
    setOutput("planned_path", path);
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "RunPlanner (stub): produced 2-point path");
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

class ValidateTrajectory : public BT::SyncActionNode
{
public:
  ValidateTrajectory(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = get_shared_node();
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<std::vector<geometry_msgs::msg::Point>>("path"),
      BT::OutputPort<bool>("ok")
    };
  }

  BT::NodeStatus tick() override
  {
    // Stub: accept any path.
    setOutput("ok", true);
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "ValidateTrajectory (stub): accepting path");
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

class ExecuteTrajectory : public BT::SyncActionNode
{
public:
  ExecuteTrajectory(const std::string & name, const BT::NodeConfiguration & config)
  : BT::SyncActionNode(name, config)
  {
    node_ = get_shared_node();
  }

  static BT::PortsList providedPorts()
  {
    return { BT::InputPort<std::vector<geometry_msgs::msg::Point>>("path") };
  }

  BT::NodeStatus tick() override
  {
    std::vector<geometry_msgs::msg::Point> path;
    getInput("path", path);
    if (node_) {
      RCLCPP_INFO(node_->get_logger(), "ExecuteTrajectory (stub): received path with %zu points", path.size());
    }
    return BT::NodeStatus::SUCCESS;
  }

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace wato::world_modeling::behaviour

// Register nodes in a single translation unit to avoid duplicate registration symbols
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<wato::world_modeling::behaviour::GetHDMapInfo>("GetHDMapInfo");
  factory.registerNodeType<wato::world_modeling::behaviour::RequestRoute>("RequestRoute");
  factory.registerNodeType<wato::world_modeling::behaviour::WaitForRoute>("WaitForRoute");
  factory.registerNodeType<wato::world_modeling::behaviour::MonitorTrafficLights>("MonitorTrafficLights");
  factory.registerNodeType<wato::world_modeling::behaviour::CheckHaveValidTrajectory>("CheckHaveValidTrajectory");
  factory.registerNodeType<wato::world_modeling::behaviour::RunPlanner>("RunPlanner");
  factory.registerNodeType<wato::world_modeling::behaviour::ValidateTrajectory>("ValidateTrajectory");
  factory.registerNodeType<wato::world_modeling::behaviour::ExecuteTrajectory>("ExecuteTrajectory");
}
