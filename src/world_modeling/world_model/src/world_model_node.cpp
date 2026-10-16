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
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

// Publishers
#include "world_model/interfaces/publishers/area_occupancy_publisher.hpp"
#include "world_model/interfaces/publishers/lane_context_publisher.hpp"
#include "world_model/interfaces/publishers/lanelet_ahead_publisher.hpp"
#include "world_model/interfaces/publishers/map_viz_publisher.hpp"
#include "world_model/interfaces/publishers/route_ahead_publisher.hpp"
#include "world_model/interfaces/publishers/world_objects_enriched_publisher.hpp"

// Services
#include "world_model/interfaces/services/get_area_occupancy_service.hpp"
#include "world_model/interfaces/services/get_lanelet_ahead_service.hpp"
#include "world_model/interfaces/services/get_objects_by_lanelet_service.hpp"
#include "world_model/interfaces/services/get_world_objects_enriched_service.hpp"
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
  this->declare_parameter<std::string>("base_frame", "base_footprint");
  this->declare_parameter<std::string>("projector_type", "utm");
  this->declare_parameter<std::string>("alignment_mode", "fixed");
  this->declare_parameter<std::string>("projected_frame", "utm");
  this->declare_parameter<double>("origin_lat", 0.0);
  this->declare_parameter<double>("origin_lon", 0.0);
  this->declare_parameter<double>("origin_alt", 0.0);

  RCLCPP_INFO(this->get_logger(), "WorldModelNode created (unconfigured)");
}

WorldModelNode::CallbackReturn WorldModelNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  RCLCPP_INFO(this->get_logger(), "Configuring...");

  // Get frame parameters
  osm_map_path_ = this->get_parameter("osm_map_path").as_string();
  map_frame_ = this->get_parameter("map_frame").as_string();
  base_frame_ = this->get_parameter("base_frame").as_string();
  map_config_.projector_type = this->get_parameter("projector_type").as_string();
  map_config_.alignment_mode = this->get_parameter("alignment_mode").as_string();
  map_config_.projected_frame = this->get_parameter("projected_frame").as_string();
  map_config_.origin_lat = this->get_parameter("origin_lat").as_double();
  map_config_.origin_lon = this->get_parameter("origin_lon").as_double();
  map_config_.origin_alt = this->get_parameter("origin_alt").as_double();
  try {
    validateMapConfig(map_config_);
    if (map_frame_.empty() || base_frame_.empty()) {
      throw std::invalid_argument("map_frame and base_frame must be nonempty");
    }
    if (map_config_.alignment_mode == "tf" && map_config_.projected_frame == map_frame_) {
      throw std::invalid_argument("TF alignment requires different projected_frame and map_frame values");
    }
  } catch (const std::exception & error) {
    RCLCPP_ERROR(this->get_logger(), "Invalid map configuration: %s", error.what());
    return CallbackReturn::FAILURE;
  }

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

  interfaces_.push_back(
    std::make_unique<WorldObjectsEnrichedPublisher>(this, world_state_.get(), lanelet_handler_.get()));

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
    std::make_unique<GetWorldObjectsEnrichedService>(this, world_state_.get(), lanelet_handler_.get()));

  interfaces_.push_back(
    std::make_unique<GetAreaOccupancyService>(this, world_state_.get(), tf_buffer_.get(), lanelet_handler_.get()));

  interfaces_.push_back(std::make_unique<GetLaneletAheadService>(this, lanelet_handler_.get()));

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
    if (map_config_.alignment_mode == "tf") {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for %s -> %s transform to align map...",
        map_config_.projected_frame.c_str(),
        map_frame_.c_str());
    } else {
      RCLCPP_INFO(this->get_logger(), "Loading map with configured projection; no UTM transform required");
    }
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

  std::optional<geometry_msgs::msg::Transform> projected_to_map;
  if (map_config_.alignment_mode == "tf") {
    try {
      // TF's target/source order returns the complete projected-frame-to-map
      // operation. Apply it to every OSM point during projection.
      const auto transform = tf_buffer_->lookupTransform(map_frame_, map_config_.projected_frame, tf2::TimePointZero);
      projected_to_map = transform.transform;
      RCLCPP_INFO(
        this->get_logger(),
        "Applying %s -> %s alignment to OSM points",
        map_config_.projected_frame.c_str(),
        map_frame_.c_str());
    } catch (const tf2::TransformException &) {
      // Transform not available yet, will retry
      RCLCPP_DEBUG_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        5000,
        "Waiting for %s -> %s transform...",
        map_config_.projected_frame.c_str(),
        map_frame_.c_str());
      return;
    }
  }

  RCLCPP_INFO(
    this->get_logger(),
    "Loading map from: %s (projector: %s, alignment: %s)",
    osm_map_path_.c_str(),
    map_config_.projector_type.c_str(),
    map_config_.alignment_mode.c_str());
  if (lanelet_handler_->loadMap(osm_map_path_, map_config_, projected_to_map)) {
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
