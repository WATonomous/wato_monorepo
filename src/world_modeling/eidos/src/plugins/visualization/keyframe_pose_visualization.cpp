#include "eidos/plugins/visualization/keyframe_pose_visualization.hpp"

#include <pluginlib/class_list_macros.hpp>

#include "eidos/slam_core.hpp"

namespace eidos {

void KeyframePoseVisualization::onInitialize() {
  RCLCPP_INFO(node_->get_logger(), "[%s] onInitialize()", name_.c_str());

  std::string prefix = name_;

  node_->declare_parameter(prefix + ".topic", "slam/visualization/poses");
  node_->declare_parameter(prefix + ".axis_length", 0.5);
  node_->declare_parameter(prefix + ".axis_width", 0.05);

  std::string topic;
  node_->get_parameter(prefix + ".topic", topic);
  node_->get_parameter(prefix + ".axis_length", axis_length_);
  node_->get_parameter(prefix + ".axis_width", axis_width_);

  node_->get_parameter("frames.map", map_frame_);

  pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>(topic, 1);

  RCLCPP_INFO(node_->get_logger(), "[%s] initialized", name_.c_str());
}

void KeyframePoseVisualization::activate() {
  active_ = true;
  pub_->on_activate();
  RCLCPP_INFO(node_->get_logger(), "[%s] activated", name_.c_str());
}

void KeyframePoseVisualization::deactivate() {
  active_ = false;
  pub_->on_deactivate();
  RCLCPP_INFO(node_->get_logger(), "[%s] deactivated", name_.c_str());
}

void KeyframePoseVisualization::onOptimizationComplete(
    const gtsam::Values& /*optimized_values*/, bool /*loop_closure_detected*/) {
  if (!active_) return;
  if (pub_->get_subscription_count() == 0) return;

  const auto& map_manager = core_->getMapManager();
  auto key_poses_6d = map_manager.getKeyPoses6D();
  int num_keyframes = map_manager.numKeyframes();

  visualization_msgs::msg::MarkerArray marker_array;

  // DELETEALL marker to clear previous markers
  visualization_msgs::msg::Marker delete_marker;
  delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
  marker_array.markers.push_back(delete_marker);

  auto stamp = node_->now();
  int marker_id = 0;

  for (int i = 0; i < num_keyframes; i++) {
    auto& pose = key_poses_6d->points[i];
    Eigen::Affine3f affine = poseTypeToAffine3f(pose);
    Eigen::Matrix3f rot = affine.rotation();

    // Origin point
    geometry_msgs::msg::Point origin;
    origin.x = pose.x;
    origin.y = pose.y;
    origin.z = pose.z;

    // X axis (red)
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = map_frame_;
      arrow.header.stamp = stamp;
      arrow.ns = "pose_axes";
      arrow.id = marker_id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point end;
      end.x = origin.x + axis_length_ * rot(0, 0);
      end.y = origin.y + axis_length_ * rot(1, 0);
      end.z = origin.z + axis_length_ * rot(2, 0);

      arrow.points.push_back(origin);
      arrow.points.push_back(end);
      arrow.scale.x = axis_width_;       // shaft diameter
      arrow.scale.y = axis_width_ * 2.0; // head diameter
      arrow.scale.z = 0.0;
      arrow.color.r = 1.0f;
      arrow.color.g = 0.0f;
      arrow.color.b = 0.0f;
      arrow.color.a = 1.0f;
      marker_array.markers.push_back(arrow);
    }

    // Y axis (green)
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = map_frame_;
      arrow.header.stamp = stamp;
      arrow.ns = "pose_axes";
      arrow.id = marker_id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point end;
      end.x = origin.x + axis_length_ * rot(0, 1);
      end.y = origin.y + axis_length_ * rot(1, 1);
      end.z = origin.z + axis_length_ * rot(2, 1);

      arrow.points.push_back(origin);
      arrow.points.push_back(end);
      arrow.scale.x = axis_width_;
      arrow.scale.y = axis_width_ * 2.0;
      arrow.scale.z = 0.0;
      arrow.color.r = 0.0f;
      arrow.color.g = 1.0f;
      arrow.color.b = 0.0f;
      arrow.color.a = 1.0f;
      marker_array.markers.push_back(arrow);
    }

    // Z axis (blue)
    {
      visualization_msgs::msg::Marker arrow;
      arrow.header.frame_id = map_frame_;
      arrow.header.stamp = stamp;
      arrow.ns = "pose_axes";
      arrow.id = marker_id++;
      arrow.type = visualization_msgs::msg::Marker::ARROW;
      arrow.action = visualization_msgs::msg::Marker::ADD;

      geometry_msgs::msg::Point end;
      end.x = origin.x + axis_length_ * rot(0, 2);
      end.y = origin.y + axis_length_ * rot(1, 2);
      end.z = origin.z + axis_length_ * rot(2, 2);

      arrow.points.push_back(origin);
      arrow.points.push_back(end);
      arrow.scale.x = axis_width_;
      arrow.scale.y = axis_width_ * 2.0;
      arrow.scale.z = 0.0;
      arrow.color.r = 0.0f;
      arrow.color.g = 0.0f;
      arrow.color.b = 1.0f;
      arrow.color.a = 1.0f;
      marker_array.markers.push_back(arrow);
    }
  }

  pub_->publish(marker_array);
}

}  // namespace eidos

PLUGINLIB_EXPORT_CLASS(eidos::KeyframePoseVisualization, eidos::VisualizationPlugin)
