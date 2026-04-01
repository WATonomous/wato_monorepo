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

#include "costmap/layers/objects_layer.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "costmap/costmap_utils.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

namespace costmap
{

void ObjectsLayer::configure(
  rclcpp_lifecycle::LifecycleNode * node, const std::string & layer_name, tf2_ros::Buffer * tf_buffer)
{
  node_ = node;
  layer_name_ = layer_name;
  tf_buffer_ = tf_buffer;

  node_->declare_parameter("layers." + layer_name_ + ".bbox_inflation_m", 0.5);
  node_->declare_parameter("layers." + layer_name_ + ".bbox_cost_decay", 1.0);
  node_->declare_parameter("layers." + layer_name_ + ".prediction_inflation_m", 1.0);
  node_->declare_parameter("layers." + layer_name_ + ".prediction_cost_decay", 0.3);
  node_->declare_parameter("layers." + layer_name_ + ".max_centroid_height_m", 100.0);

  bbox_inflation_m_ = node_->get_parameter("layers." + layer_name_ + ".bbox_inflation_m").as_double();
  bbox_cost_decay_ = node_->get_parameter("layers." + layer_name_ + ".bbox_cost_decay").as_double();
  prediction_inflation_m_ = node_->get_parameter("layers." + layer_name_ + ".prediction_inflation_m").as_double();
  prediction_cost_decay_ = node_->get_parameter("layers." + layer_name_ + ".prediction_cost_decay").as_double();
  max_centroid_height_m_ = node_->get_parameter("layers." + layer_name_ + ".max_centroid_height_m").as_double();
}

void ObjectsLayer::activate()
{
  objects_sub_ = node_->create_subscription<world_model_msgs::msg::WorldObjectArray>(
    "world_objects", rclcpp::QoS(10), std::bind(&ObjectsLayer::objectsCallback, this, std::placeholders::_1));
}

void ObjectsLayer::deactivate()
{
  objects_sub_.reset();
}

void ObjectsLayer::cleanup()
{
  objects_sub_.reset();
  latest_objects_.reset();
}

void ObjectsLayer::objectsCallback(const world_model_msgs::msg::WorldObjectArray::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(data_mutex_);
  latest_objects_ = msg;
}

void ObjectsLayer::update(
  nav_msgs::msg::OccupancyGrid & grid, const geometry_msgs::msg::TransformStamped & /*map_to_costmap*/)
{
  world_model_msgs::msg::WorldObjectArray::SharedPtr objects;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    objects = latest_objects_;
  }

  if (!objects) {
    return;
  }

  // Look up transform from the objects' frame to costmap frame
  geometry_msgs::msg::TransformStamped obj_to_costmap;
  try {
    obj_to_costmap = tf_buffer_->lookupTransform(grid.header.frame_id, objects->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(), *node_->get_clock(), 2000, "ObjectsLayer TF lookup failed: %s", ex.what());
    return;
  }

  for (const auto & obj : objects->objects) {
    const auto & det = obj.detection;
    const auto & bbox = det.bbox;

    // Skip objects whose centroid is above the max height (e.g. traffic lights)
    if (bbox.center.position.z > max_centroid_height_m_) {
      continue;
    }

    // Transform bbox center to costmap frame
    geometry_msgs::msg::PoseStamped pose_in, pose_out;
    pose_in.pose = bbox.center;
    tf2::doTransform(pose_in, pose_out, obj_to_costmap);

    double obj_yaw = yawFromQuat(pose_out.pose.orientation);
    double base_half_x = bbox.size.x / 2.0;
    double base_half_y = bbox.size.y / 2.0;

    // Mark inflated region at decayed cost, then original bbox at full cost
    if (bbox_inflation_m_ > 0.0) {
      int8_t inflated_cost = static_cast<int8_t>(std::max(1.0, 100.0 * bbox_cost_decay_));
      markBox(
        grid,
        pose_out.pose.position.x,
        pose_out.pose.position.y,
        obj_yaw,
        base_half_x + bbox_inflation_m_,
        base_half_y + bbox_inflation_m_,
        inflated_cost);
    }

    markBox(grid, pose_out.pose.position.x, pose_out.pose.position.y, obj_yaw, base_half_x, base_half_y, 100);

    // Project prediction poses with distance-based cost decay
    // Close predictions have cost nearly equal to the full vehicle
    // Low probability predictions decay faster with distance
    for (const auto & prediction : obj.predictions) {
      for (const auto & ps : prediction.poses) {
        geometry_msgs::msg::PoseStamped pred_out;
        tf2::doTransform(ps, pred_out, obj_to_costmap);

        // Decay along the object's predicted path, not by global ego distance.
        // This preserves visible prediction costs even for distant objects.
        double dx = pred_out.pose.position.x - pose_out.pose.position.x;
        double dy = pred_out.pose.position.y - pose_out.pose.position.y;
        double distance = std::sqrt(dx * dx + dy * dy);

        // Distance-based decay: low confidence predictions decay faster.
        // The configured base decay is scaled inversely by confidence.
        double conf_clamped = std::max(0.1, prediction.conf);
        double decay_rate = prediction_cost_decay_ / conf_clamped;
        double distance_decay = std::exp(-decay_rate * distance);

        // Skip cells whose decayed value is effectively free space.
        double raw_pred_cost = 100.0 * prediction.conf * distance_decay;
        if (raw_pred_cost < 1.0) {
          continue;
        }
        int8_t pred_cost = static_cast<int8_t>(std::min(100.0, raw_pred_cost));

        double pred_yaw = yawFromQuat(pred_out.pose.orientation);

        // Mark inflated prediction region at decayed cost, then core at full pred cost
        if (prediction_inflation_m_ > 0.0) {
          int8_t inflated_pred_cost = static_cast<int8_t>(std::max(1.0, pred_cost * bbox_cost_decay_));
          markBox(
            grid,
            pred_out.pose.position.x,
            pred_out.pose.position.y,
            pred_yaw,
            base_half_x + prediction_inflation_m_,
            base_half_y + prediction_inflation_m_,
            inflated_pred_cost);
        }

        markBox(
          grid,
          pred_out.pose.position.x,
          pred_out.pose.position.y,
          pred_yaw,
          base_half_x,
          base_half_y,
          pred_cost);
      }
    }
  }
}

}  // namespace costmap

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(costmap::ObjectsLayer, costmap::CostmapLayer)
