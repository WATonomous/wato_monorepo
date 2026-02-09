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

#include "world_model/world_model_node.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

// Publishers
#include "world_model/interfaces/publishers/area_occupancy_publisher.hpp"
#include "world_model/interfaces/publishers/dynamic_objects_publisher.hpp"
#include "world_model/interfaces/publishers/lane_context_publisher.hpp"
#include "world_model/interfaces/publishers/lanelet_ahead_publisher.hpp"
#include "world_model/interfaces/publishers/map_viz_publisher.hpp"
#include "world_model/interfaces/publishers/route_ahead_publisher.hpp"

// Services
#include "world_model/interfaces/services/get_area_occupancy_service.hpp"
#include "world_model/interfaces/services/get_dynamic_objects_service.hpp"
#include "world_model/interfaces/services/get_objects_by_lanelet_service.hpp"
#include "world_model/interfaces/services/reg_elem_service.hpp"
#include "world_model/interfaces/services/set_route_service.hpp"
#include "world_model/interfaces/services/shortest_route_service.hpp"

namespace world_model
{

WorldModelNode::WorldModelNode(const rclcpp::NodeOptions & options)
: LifecycleNode("world_model", options)
{
  // Declare shared parameters (used by both the node and interfaces)
  this->declare_parameter<std::string>("osm_map_path", "");
  this->declare_parameter<std::string>("map_frame", "map");
  this->declare_parameter<std::string>("base_frame", "base_link");
  this->declare_parameter<std::string>("utm_frame", "utm");
  this->declare_parameter<std::string>("projector_type", "utm");

  RCLCPP_INFO(this->get_logger(), "WorldModelNode created (unconfigured)");
}

WorldModelNode::CallbackReturn WorldModelNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  // Get frame parameters
  osm_map_path_ = this->get_parameter("osm_map_path").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  utm_frame_ = this->get_parameter("utm_frame").as_string();
  projector_type_ = this->get_parameter("projector_type").as_string();

  // Initialize TF
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Initialize core components
  world_state_ = std::make_unique<WorldState>();
  lanelet_handler_ = std::make_unique<LaneletHandler>();

  // Create all interface components
  createInterfaces();

  RCLCPP_INFO(this->get_logger(), "Configured successfully");
  return CallbackReturn::SUCCESS;
}

void WorldModelNode::createInterfaces()
{
  // Publishers
  interfaces_.push_back(std::make_unique<LaneContextPublisher>(this, lanelet_handler_.get(), tf_buffer_.get()));

  interfaces_.push_back(std::make_unique<MapVizPublisher>(this, lanelet_handler_.get(), tf_buffer_.get()));

  interfaces_.push_back(std::make_unique<DynamicObjectsPublisher>(this, world_state_.get(), lanelet_handler_.get()));

  interfaces_.push_back(std::make_unique<RouteAheadPublisher>(this, lanelet_handler_.get(), tf_buffer_.get()));

  interfaces_.push_back(
    std::make_unique<AreaOccupancyPublisher>(this, world_state_.get(), tf_buffer_.get(), lanelet_handler_.get()));

  interfaces_.push_back(std::make_unique<LaneletAheadPublisher>(this, lanelet_handler_.get(), tf_buffer_.get()));

  // Services
  interfaces_.push_back(std::make_unique<SetRouteService>(this, lanelet_handler_.get(), tf_buffer_.get()));

  interfaces_.push_back(std::make_unique<ShortestRouteService>(this, lanelet_handler_.get(), tf_buffer_.get()));

  interfaces_.push_back(std::make_unique<RegElemService>(this, lanelet_handler_.get()));

  interfaces_.push_back(std::make_unique<GetObjectsByLaneletService>(this, world_state_.get(), lanelet_handler_.get()));

  interfaces_.push_back(
    std::make_unique<GetDynamicObjectsService>(this, world_state_.get(), lanelet_handler_.get()));

  interfaces_.push_back(
    std::make_unique<GetAreaOccupancyService>(this, world_state_.get(), tf_buffer_.get(), lanelet_handler_.get()));

  // Single inbound subscriber
  writer_ = std::make_unique<WorldModelWriter>(this, world_state_.get(), lanelet_handler_.get(), tf_buffer_.get());
}

WorldModelNode::CallbackReturn WorldModelNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Activating...");

  // Activate all interface components (publishers, subscribers, services, workers)
  for (auto & interface : interfaces_) {
    interface->activate();
  }

  // Start map loading timer
  if (!osm_map_path_.empty()) {
    RCLCPP_INFO(
      this->get_logger(), "Waiting for %s -> %s transform to load map...", utm_frame_.c_str(), map_frame_.c_str());
    map_init_timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&WorldModelNode::tryLoadMap, this));
  } else {
    RCLCPP_WARN(this->get_logger(), "No map path provided, services will return errors");
  }

  RCLCPP_INFO(this->get_logger(), "Activated successfully");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn WorldModelNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating...");

  // Deactivate all interface components (publishers, subscribers, services, workers)
  for (auto & interface : interfaces_) {
    interface->deactivate();
  }

  // Cancel map init timer
  if (map_init_timer_) {
    map_init_timer_->cancel();
    map_init_timer_.reset();
  }

  RCLCPP_INFO(this->get_logger(), "Deactivated successfully");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn WorldModelNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up...");

  // Clear subscriber and interfaces
  writer_.reset();
  interfaces_.clear();

  // Reset other resources
  tf_listener_.reset();
  tf_buffer_.reset();
  lanelet_handler_.reset();
  world_state_.reset();

  RCLCPP_INFO(this->get_logger(), "Cleaned up successfully");
  return CallbackReturn::SUCCESS;
}

WorldModelNode::CallbackReturn WorldModelNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down...");

  // Deactivate interfaces (safe even if never activated — methods have null guards)
  for (auto & interface : interfaces_) {
    interface->deactivate();
  }

  if (map_init_timer_) {
    map_init_timer_->cancel();
    map_init_timer_.reset();
  }

  // Release all resources
  writer_.reset();
  interfaces_.clear();
  tf_listener_.reset();
  tf_buffer_.reset();
  lanelet_handler_.reset();
  world_state_.reset();

  return CallbackReturn::SUCCESS;
}

void WorldModelNode::tryLoadMap()
{
  if (lanelet_handler_->isMapLoaded()) {
    map_init_timer_->cancel();
    return;
  }

  double utm_origin_x = 0.0;
  double utm_origin_y = 0.0;

  // For local_cartesian projector (CARLA/simulation), use (0,0) offset directly
  // For utm projector (real-world), wait for TF to determine the offset
  if (projector_type_ == "local_cartesian") {
    RCLCPP_INFO(this->get_logger(), "Using local_cartesian projector with origin offset: (0.0, 0.0)");
  } else {
    try {
      auto transform = tf_buffer_->lookupTransform(map_frame_, utm_frame_, tf2::TimePointZero);

      // The translation represents where utm(0,0,0) is in the map frame
      // We want the inverse: where map(0,0,0) is in utm frame
      utm_origin_x = -transform.transform.translation.x;
      utm_origin_y = -transform.transform.translation.y;

      RCLCPP_INFO(
        this->get_logger(),
        "Got %s -> %s transform. UTM origin offset: (%.2f, %.2f)",
        utm_frame_.c_str(),
        map_frame_.c_str(),
        utm_origin_x,
        utm_origin_y);
    } catch (const tf2::TransformException &) {
      // Transform not available yet, will retry
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Waiting for %s -> %s transform...",
        utm_frame_.c_str(),
        map_frame_.c_str());
      return;
    }
  }

  RCLCPP_INFO(
    this->get_logger(), "Loading map from: %s (projector: %s)", osm_map_path_.c_str(), projector_type_.c_str());
  if (lanelet_handler_->loadMap(osm_map_path_, utm_origin_x, utm_origin_y, projector_type_)) {
    RCLCPP_INFO(this->get_logger(), "Map loaded successfully");
    map_init_timer_->cancel();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to load map");
    map_init_timer_->cancel();
  }
}
}  // namespace world_model

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<world_model::WorldModelNode>(rclcpp::NodeOptions());

  // Use MultiThreadedExecutor for callback groups to work properly
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());

  executor.spin();

  rclcpp::shutdown();
  return 0;
}
