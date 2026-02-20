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

#include "costmap/costmap_node.hpp"

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"

namespace costmap
{

CostmapNode::CostmapNode(const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode("costmap_node", options)
, layer_loader_("costmap", "costmap::CostmapLayer")
{
  declare_parameter("costmap_frame", "base_footprint");
  declare_parameter("map_frame", "map");
  declare_parameter("publish_rate_hz", 20.0);
  declare_parameter("grid_width_m", 60.0);
  declare_parameter("grid_height_m", 60.0);
  declare_parameter("resolution", 0.25);
  declare_parameter("layers", std::vector<std::string>{"objects", "virtual_wall"});
  declare_parameter("footprint_front_left", std::vector<double>{0.0, 0.0});
  declare_parameter("footprint_rear_right", std::vector<double>{0.0, 0.0});
}

CostmapNode::CallbackReturn CostmapNode::on_configure(const rclcpp_lifecycle::State & /*state*/)
{
  costmap_frame_ = get_parameter("costmap_frame").as_string();
  map_frame_ = get_parameter("map_frame").as_string();
  publish_rate_hz_ = get_parameter("publish_rate_hz").as_double();
  grid_width_m_ = get_parameter("grid_width_m").as_double();
  grid_height_m_ = get_parameter("grid_height_m").as_double();
  resolution_ = get_parameter("resolution").as_double();
  layer_names_ = get_parameter("layers").as_string_array();
  footprint_front_left_ = get_parameter("footprint_front_left").as_double_array();
  footprint_rear_right_ = get_parameter("footprint_rear_right").as_double_array();

  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", rclcpp::QoS(10));
  footprint_pub_ = create_publisher<geometry_msgs::msg::PolygonStamped>("footprint", rclcpp::QoS(10));

  for (const auto & name : layer_names_) {
    std::string type_param = "layers." + name + ".type";
    if (!has_parameter(type_param)) {
      declare_parameter(type_param, "");
    }
    std::string plugin_type = get_parameter(type_param).as_string();
    if (plugin_type.empty()) {
      RCLCPP_ERROR(get_logger(), "No plugin type specified for layer '%s'", name.c_str());
      return CallbackReturn::FAILURE;
    }
    try {
      auto layer = layer_loader_.createSharedInstance(plugin_type);
      layer->configure(this, name, tf_buffer_.get());
      layers_.push_back(std::move(layer));
    } catch (const pluginlib::PluginlibException & ex) {
      RCLCPP_ERROR(get_logger(), "Failed to load layer plugin '%s': %s", plugin_type.c_str(), ex.what());
      return CallbackReturn::FAILURE;
    }
  }

  RCLCPP_INFO(
    get_logger(),
    "Configured with %zu layers, %.2f Hz, %.1f x %.1f m @ %.2f m/cell",
    layers_.size(),
    publish_rate_hz_,
    grid_width_m_,
    grid_height_m_,
    resolution_);

  return CallbackReturn::SUCCESS;
}

CostmapNode::CallbackReturn CostmapNode::on_activate(const rclcpp_lifecycle::State & /*state*/)
{
  costmap_pub_->on_activate();
  footprint_pub_->on_activate();

  for (auto & layer : layers_) {
    layer->activate();
  }

  const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz_);
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period), std::bind(&CostmapNode::publishCostmap, this));

  RCLCPP_INFO(get_logger(), "Activated");
  return CallbackReturn::SUCCESS;
}

CostmapNode::CallbackReturn CostmapNode::on_deactivate(const rclcpp_lifecycle::State & /*state*/)
{
  publish_timer_.reset();

  for (auto & layer : layers_) {
    layer->deactivate();
  }

  costmap_pub_->on_deactivate();
  footprint_pub_->on_deactivate();

  RCLCPP_INFO(get_logger(), "Deactivated");
  return CallbackReturn::SUCCESS;
}

CostmapNode::CallbackReturn CostmapNode::on_cleanup(const rclcpp_lifecycle::State & /*state*/)
{
  for (auto & layer : layers_) {
    layer->cleanup();
  }
  layers_.clear();
  layer_names_.clear();

  costmap_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  RCLCPP_INFO(get_logger(), "Cleaned up");
  return CallbackReturn::SUCCESS;
}

CostmapNode::CallbackReturn CostmapNode::on_shutdown(const rclcpp_lifecycle::State & /*state*/)
{
  publish_timer_.reset();
  for (auto & layer : layers_) {
    layer->cleanup();
  }
  layers_.clear();

  costmap_pub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();

  RCLCPP_INFO(get_logger(), "Shut down");
  return CallbackReturn::SUCCESS;
}

void CostmapNode::publishCostmap()
{
  geometry_msgs::msg::TransformStamped map_to_costmap;
  try {
    map_to_costmap = tf_buffer_->lookupTransform(costmap_frame_, map_frame_, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "TF lookup failed: %s", ex.what());
    return;
  }

  nav_msgs::msg::OccupancyGrid grid;
  grid.header.stamp = now();
  grid.header.frame_id = costmap_frame_;
  grid.info.resolution = static_cast<float>(resolution_);
  grid.info.width = static_cast<uint32_t>(grid_width_m_ / resolution_);
  grid.info.height = static_cast<uint32_t>(grid_height_m_ / resolution_);

  // Center grid on the costmap frame origin
  grid.info.origin.position.x = -grid_width_m_ / 2.0;
  grid.info.origin.position.y = -grid_height_m_ / 2.0;
  grid.info.origin.orientation.w = 1.0;

  grid.data.assign(grid.info.width * grid.info.height, 0);

  for (auto & layer : layers_) {
    layer->update(grid, map_to_costmap);
  }

  costmap_pub_->publish(grid);

  // Publish footprint polygon (front_left and rear_right define opposite corners)
  if (
    footprint_front_left_.size() == 2 && footprint_rear_right_.size() == 2 &&
    (footprint_front_left_[0] != footprint_rear_right_[0] || footprint_front_left_[1] != footprint_rear_right_[1]))
  {
    geometry_msgs::msg::PolygonStamped footprint;
    footprint.header.stamp = grid.header.stamp;
    footprint.header.frame_id = costmap_frame_;

    const auto fl_x = static_cast<float>(footprint_front_left_[0]);
    const auto fl_y = static_cast<float>(footprint_front_left_[1]);
    const auto rr_x = static_cast<float>(footprint_rear_right_[0]);
    const auto rr_y = static_cast<float>(footprint_rear_right_[1]);

    geometry_msgs::msg::Point32 pt;
    pt.z = 0.0f;

    pt.x = fl_x;
    pt.y = fl_y;
    footprint.polygon.points.push_back(pt);  // Front-left

    pt.x = fl_x;
    pt.y = rr_y;
    footprint.polygon.points.push_back(pt);  // Front-right

    pt.x = rr_x;
    pt.y = rr_y;
    footprint.polygon.points.push_back(pt);  // Rear-right

    pt.x = rr_x;
    pt.y = fl_y;
    footprint.polygon.points.push_back(pt);  // Rear-left

    footprint_pub_->publish(footprint);
  }
}

}  // namespace costmap

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(costmap::CostmapNode)
