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

#include "tracking/tracking.hpp"

#include <algorithm>
#include <exception>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <tf2/utils.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <world_model_msgs/msg/prediction.hpp>
#include <world_model_msgs/msg/world_object.hpp>

// static logger for static logging
rclcpp::Logger TrackingNode::static_logger_ = rclcpp::get_logger("tracking_stc");

TrackingNode::TrackingNode(const rclcpp::NodeOptions & options)
: LifecycleNode("tracking", options)
{
  RCLCPP_INFO(this->get_logger(), "New tracking node created");
}

TrackingNode::CallbackReturn TrackingNode::on_configure(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Configuring tracking node...");

  // Parameters
  initializeParams();

  // TF stuff
  tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Subscriber
  dets_sub_ = this->create_subscription<vision_msgs::msg::Detection3DArray>(
    kDetectionsTopic, 10, std::bind(&TrackingNode::detectionsCallback, this, std::placeholders::_1));
  RCLCPP_DEBUG(this->get_logger(), "Subscriber initialized, subscribed to %s", dets_sub_->get_topic_name());

  // Lifecycle Publishers
  tracked_dets_pub_ = this->create_publisher<vision_msgs::msg::Detection3DArray>(kTracksTopic, 10);
  markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(kMarkersTopic, 10);
  predictions_pub_ = this->create_publisher<world_model_msgs::msg::WorldObjectArray>(kPredictionsTopic, 10);
  RCLCPP_DEBUG(this->get_logger(), "Publishers initialized, publishing to %s", tracked_dets_pub_->get_topic_name());

  // ByteTrack tracker
  tracker_ = std::make_unique<byte_track::BYTETracker>(
    frame_rate_, track_buffer_, track_thresh_, high_thresh_, match_thresh_, use_maj_cls_);

  RCLCPP_INFO(this->get_logger(), "Configuration successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

TrackingNode::CallbackReturn TrackingNode::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Activating tracking node...");
  if (tracked_dets_pub_ && markers_pub_ && predictions_pub_) {
    tracked_dets_pub_->on_activate();
    markers_pub_->on_activate();
    predictions_pub_->on_activate();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Activation failed: nullptr publisher");
    return TrackingNode::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "Activation successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

TrackingNode::CallbackReturn TrackingNode::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Deactivating tracking node...");
  if (tracked_dets_pub_ && markers_pub_ && predictions_pub_) {
    tracked_dets_pub_->on_deactivate();
    markers_pub_->on_deactivate();
    predictions_pub_->on_deactivate();
  } else {
    RCLCPP_ERROR(this->get_logger(), "Deactivation failed: nullptr publisher");
    return TrackingNode::CallbackReturn::FAILURE;
  }

  RCLCPP_INFO(this->get_logger(), "Deactivation successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

TrackingNode::CallbackReturn TrackingNode::on_cleanup(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Cleaning up tracking node...");

  tracked_dets_pub_.reset();
  markers_pub_.reset();
  predictions_pub_.reset();
  dets_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tracker_.reset();
  track_filters_.clear();
  track_centroids_.clear();

  RCLCPP_INFO(this->get_logger(), "Clean up successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

TrackingNode::CallbackReturn TrackingNode::on_shutdown(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(this->get_logger(), "Shutting down tracking node...");

  tracked_dets_pub_.reset();
  markers_pub_.reset();
  predictions_pub_.reset();
  dets_sub_.reset();
  tf_listener_.reset();
  tf_buffer_.reset();
  tracker_.reset();
  track_filters_.clear();
  track_centroids_.clear();

  RCLCPP_INFO(this->get_logger(), "Shut down successful");
  return TrackingNode::CallbackReturn::SUCCESS;
}

void TrackingNode::initializeParams()
{
  // Declare parameters
  frame_rate_ = this->declare_parameter<int>("frame_rate", 2);
  track_buffer_ = this->declare_parameter<int>("track_buffer", 30);
  track_thresh_ = static_cast<float>(this->declare_parameter<double>("track_thresh", 0.5));
  high_thresh_ = static_cast<float>(this->declare_parameter<double>("high_thresh", 0.6));
  match_thresh_ = static_cast<float>(this->declare_parameter<double>("match_thresh", 1.0));
  use_maj_cls_ = this->declare_parameter<bool>("use_maj_cls", true);
  output_frame_ = this->declare_parameter<std::string>("output_frame", "map");
  publish_visualization_ = this->declare_parameter<bool>("publish_visualization", false);

  // Prediction parameters
  centroid_history_size_ = this->declare_parameter<int>("centroid_history_size", 10);
  prediction_time_ = this->declare_parameter<double>("prediction_time", 5.0);
  prediction_dt_ = this->declare_parameter<double>("prediction_dt", 0.1);
  process_noise_ = this->declare_parameter<double>("process_noise", 0.1);
  measurement_noise_ = this->declare_parameter<double>("measurement_noise", 0.5);
}

// Convert from ros msgs to bytetrack's required format
std::vector<byte_track::Object> TrackingNode::detsToObjects(const vision_msgs::msg::Detection3DArray & dets)
{
  std::vector<byte_track::Object> objs;
  objs.reserve(dets.detections.size());

  for (const auto & det : dets.detections) {
    // Convert from Detection3D to byte_track::Object
    float length = det.bbox.size.x;
    float width = det.bbox.size.y;
    float height = det.bbox.size.z;
    float yaw = tf2::getYaw(det.bbox.center.orientation);
    float x = det.bbox.center.position.x;
    float y = det.bbox.center.position.y;
    float z = det.bbox.center.position.z;

    byte_track::Rect<float> rect{x, y, z, yaw, length, width, height};
    int label = 0;
    float prob = 1.0;

    // Get highest scored class hypothesis, skipping attribute hypotheses (prefixed with "state:", "behavior:", etc.)
    decltype(det.results.begin()) best_hyp = det.results.end();
    for (auto it = det.results.begin(); it != det.results.end(); ++it) {
      const auto & cid = it->hypothesis.class_id;
      if (cid.find(':') != std::string::npos) continue;  // Skip attribute hypotheses
      if (best_hyp == det.results.end() || it->hypothesis.score > best_hyp->hypothesis.score) {
        best_hyp = it;
      }
    }

    if (best_hyp == det.results.end()) {
      RCLCPP_WARN(static_logger_, "det.results must be non-empty, falling back to dummy values");
      label = -1;
      prob = 0.0;
    } else {
      label = best_hyp->hypothesis.class_id;
      prob = best_hyp->hypothesis.score;
    }

    byte_track::Object obj(rect, label, prob);
    objs.push_back(obj);
  }

  return objs;
}

// Convert from bytetrack format back to ros msgs
vision_msgs::msg::Detection3DArray TrackingNode::STracksToTracks(
  const std::vector<byte_track::BYTETracker::STrackPtr> & strk_ptrs, const std_msgs::msg::Header & header)
{
  // Use same header as detections for same time stamps
  vision_msgs::msg::Detection3DArray trks;
  trks.header.frame_id = header.frame_id;
  trks.header.stamp = header.stamp;

  int skip_count = 0;
  for (const auto & strk_ptr : strk_ptrs) {
    // Convert STrackPtr to Detection2D
    auto rect = strk_ptr->getRect();
    auto score = strk_ptr->getScore();
    auto trk_id = strk_ptr->getTrackId();
    auto class_id = strk_ptr->getClassId();
    auto state_mean = strk_ptr->getMean();

    vision_msgs::msg::Detection3D trk;
    trk.header.frame_id = header.frame_id;
    trk.header.stamp = header.stamp;

    vision_msgs::msg::ObjectHypothesisWithPose hyp;
    hyp.hypothesis.score = score;
    hyp.hypothesis.class_id = class_id;
    trk.results.push_back(hyp);

    vision_msgs::msg::ObjectHypothesisWithPose vel;
    vel.hypothesis.class_id = "linear_velocity";
    if (state_mean.size() > 9) {
      vel.hypothesis.score = 1.0;
      vel.pose.pose.position.x = state_mean[7];
      vel.pose.pose.position.y = state_mean[8];
      vel.pose.pose.position.z = state_mean[9];
    } else {
      vel.hypothesis.score = 0.0;
      ++skip_count;
    }
    trk.results.push_back(vel);

    tf2::Quaternion q;
    q.setRPY(0, 0, rect.yaw());

    trk.bbox.center.position.x = rect.x();
    trk.bbox.center.position.y = rect.y();
    trk.bbox.center.position.z = rect.z();
    trk.bbox.center.orientation = tf2::toMsg(q);
    trk.bbox.size.x = rect.length();
    trk.bbox.size.y = rect.width();
    trk.bbox.size.z = rect.height();

    trk.id = std::to_string(trk_id);

    trks.detections.push_back(trk);
  }

  if (skip_count > 0) {
    RCLCPP_WARN(
      static_logger_,
      "Received unexpected mean shape for %d tracks, velocity hypothesis score is set to 0.0",
      skip_count);
  }

  return trks;
}

void TrackingNode::detectionsCallback(vision_msgs::msg::Detection3DArray::SharedPtr msg)
{
  // Activation guard
  if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {
    return;
  }

  // Get newest frame transform
  geometry_msgs::msg::TransformStamped tf_stamped;
  try {
    tf_stamped = tf_buffer_->lookupTransform(output_frame_, msg->header.frame_id, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_WARN(this->get_logger(), "Transform unavailable: %s", ex.what());
    RCLCPP_WARN(this->get_logger(), "Falling back to identity transform");
    // Fall back to identity
    tf_stamped.header.stamp = msg->header.stamp;
    tf_stamped.header.frame_id = output_frame_;
    tf_stamped.child_frame_id = msg->header.frame_id;
    tf_stamped.transform.translation.x = 0.0;
    tf_stamped.transform.translation.y = 0.0;
    tf_stamped.transform.translation.z = 0.0;
    tf_stamped.transform.rotation.x = 0.0;
    tf_stamped.transform.rotation.y = 0.0;
    tf_stamped.transform.rotation.z = 0.0;
    tf_stamped.transform.rotation.w = 1.0;
  }

  vision_msgs::msg::Detection3DArray tf_msg;
  tf_msg.header.frame_id = output_frame_;
  tf_msg.header.stamp = msg->header.stamp;

  // Transform all dets
  for (const auto & det : msg->detections) {
    vision_msgs::msg::Detection3D tf_det = det;
    tf_det.header.frame_id = output_frame_;
    tf2::doTransform(det.bbox.center, tf_det.bbox.center, tf_stamped);
    tf_msg.detections.push_back(tf_det);
  }

  // Run bytetrack on detections
  auto objs = detsToObjects(tf_msg);
  auto stracks = tracker_->update(objs);
  auto tracked_dets = STracksToTracks(stracks, tf_msg.header);

  // Match each tracked detection to the nearest input detection to:
  // 1. Override ByteTrack's filtered yaw (Kalman filter causes unstable spinning)
  // 2. Carry forward attribute hypotheses (state:*, behavior:*) from enriched detections
  for (auto & trk : tracked_dets.detections) {
    double best_dist = std::numeric_limits<double>::max();
    const vision_msgs::msg::Detection3D * best_det = nullptr;
    for (const auto & det : tf_msg.detections) {
      double dx = trk.bbox.center.position.x - det.bbox.center.position.x;
      double dy = trk.bbox.center.position.y - det.bbox.center.position.y;
      double dz = trk.bbox.center.position.z - det.bbox.center.position.z;
      double dist = dx * dx + dy * dy + dz * dz;
      if (dist < best_dist) {
        best_dist = dist;
        best_det = &det;
      }
    }
    if (best_det) {
      trk.bbox.center.orientation = best_det->bbox.center.orientation;

      // Append attribute hypotheses (class_id contains ':') from the matched input detection
      for (const auto & result : best_det->results) {
        if (result.hypothesis.class_id.find(':') != std::string::npos) {
          trk.results.push_back(result);
        }
      }
    }
  }

  RCLCPP_DEBUG(
    this->get_logger(),
    "Received %zd dets, transformed to %zd dets, publishing %zd tracks...",
    msg->detections.size(),
    tf_msg.detections.size(),
    tracked_dets.detections.size());
  tracked_dets_pub_->publish(tracked_dets);

  // --- Kalman filter prediction for each tracked object ---
  // Collect active track IDs so we can prune stale filters
  std::unordered_map<int, bool> active_track_ids;

  world_model_msgs::msg::WorldObjectArray world_objects;
  world_objects.header = tracked_dets.header;

  const rclcpp::Time current_time(tracked_dets.header.stamp);
  const int num_prediction_steps = static_cast<int>(prediction_time_ / prediction_dt_);

  // Kalman filter matrices (constant velocity model)
  // Measurement matrix H: observe [x, y] from state [x, y, vx, vy]
  Eigen::Matrix<double, 2, 4> H;
  H << 1, 0, 0, 0, 0, 1, 0, 0;

  // Measurement noise R
  Eigen::Matrix2d R = Eigen::Matrix2d::Identity() * measurement_noise_;

  for (const auto & trk : tracked_dets.detections) {
    int trk_id = std::stoi(trk.id);
    active_track_ids[trk_id] = true;

    double cx = trk.bbox.center.position.x;
    double cy = trk.bbox.center.position.y;

    // Store centroid in history
    geometry_msgs::msg::PoseStamped pt;
    pt.header = trk.header;
    pt.pose.position = trk.bbox.center.position;
    pt.pose.orientation = trk.bbox.center.orientation;
    auto & centroid_history = track_centroids_[trk_id];
    centroid_history.push_back(pt);
    while (static_cast<int>(centroid_history.size()) > centroid_history_size_) {
      centroid_history.pop_front();
    }

    // Initialize or update Kalman filter
    auto & kf = track_filters_[trk_id];
    if (!kf.initialized) {
      // Seed velocity from centroid history if we have at least 2 points
      double vx_init = 0.0, vy_init = 0.0;
      double vel_uncertainty = 10.0;
      if (centroid_history.size() >= 2) {
        const auto & prev = centroid_history[centroid_history.size() - 2];
        const auto & curr = centroid_history.back();
        double dt_init = (rclcpp::Time(curr.header.stamp) - rclcpp::Time(prev.header.stamp)).seconds();
        if (dt_init > 1e-6) {
          vx_init = (curr.pose.position.x - prev.pose.position.x) / dt_init;
          vy_init = (curr.pose.position.y - prev.pose.position.y) / dt_init;
          vel_uncertainty = 5.0;  // lower uncertainty when we have a velocity estimate
        }
      }
      kf.state << cx, cy, vx_init, vy_init;
      kf.covariance = Eigen::Matrix4d::Identity();
      kf.covariance(2, 2) = vel_uncertainty;
      kf.covariance(3, 3) = vel_uncertainty;
      kf.last_update_time = current_time;
      kf.initialized = true;
    } else {
      double dt = (current_time - kf.last_update_time).seconds();
      if (dt <= 0.0) dt = prediction_dt_;
      // Cap dt to prevent process noise explosion after missed detections
      dt = std::min(dt, 1.0);

      // State transition matrix F
      Eigen::Matrix4d F = Eigen::Matrix4d::Identity();
      F(0, 2) = dt;
      F(1, 3) = dt;

      // Process noise Q
      double dt2 = dt * dt;
      double dt3 = dt2 * dt / 2.0;
      double dt4 = dt2 * dt2 / 4.0;
      Eigen::Matrix4d Q;
      Q << dt4, 0, dt3, 0, 0, dt4, 0, dt3, dt3, 0, dt2, 0, 0, dt3, 0, dt2;
      Q *= process_noise_;

      // Predict step
      kf.state = F * kf.state;
      kf.covariance = F * kf.covariance * F.transpose() + Q;

      // Update step with measurement [cx, cy]
      Eigen::Vector2d z(cx, cy);
      Eigen::Vector2d y = z - H * kf.state;  // innovation
      Eigen::Matrix2d S = H * kf.covariance * H.transpose() + R;  // innovation covariance
      Eigen::Matrix<double, 4, 2> K = kf.covariance * H.transpose() * S.inverse();  // Kalman gain

      kf.state = kf.state + K * y;
      kf.covariance = (Eigen::Matrix4d::Identity() - K * H) * kf.covariance;
      kf.last_update_time = current_time;
    }

    // Generate predicted path by propagating filtered state forward
    world_model_msgs::msg::WorldObject world_obj;
    world_obj.header = tracked_dets.header;
    world_obj.detection = trk;

    world_model_msgs::msg::Prediction prediction;
    prediction.header = tracked_dets.header;
    prediction.conf = 1.0;

    Eigen::Vector4d pred_state = kf.state;
    for (int step = 0; step <= num_prediction_steps; ++step) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header.frame_id = tracked_dets.header.frame_id;
      pose.header.stamp = rclcpp::Time(current_time) + rclcpp::Duration::from_seconds(step * prediction_dt_);
      pose.pose.position.x = pred_state(0);
      pose.pose.position.y = pred_state(1);
      pose.pose.position.z = trk.bbox.center.position.z;

      // Orientation from velocity direction
      double vx = pred_state(2);
      double vy = pred_state(3);
      double yaw = std::atan2(vy, vx);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      pose.pose.orientation = tf2::toMsg(q);

      prediction.poses.push_back(pose);

      // Propagate state forward by prediction_dt_
      pred_state(0) += pred_state(2) * prediction_dt_;
      pred_state(1) += pred_state(3) * prediction_dt_;
    }

    world_obj.predictions.push_back(prediction);
    world_objects.objects.push_back(world_obj);
  }

  // Prune stale track filters and centroid histories
  for (auto it = track_filters_.begin(); it != track_filters_.end();) {
    if (active_track_ids.find(it->first) == active_track_ids.end()) {
      track_centroids_.erase(it->first);
      it = track_filters_.erase(it);
    } else {
      ++it;
    }
  }

  predictions_pub_->publish(world_objects);

  // Publish visualization markers
  if (publish_visualization_) {
    visualization_msgs::msg::MarkerArray marker_array;

    // Delete all previous markers
    visualization_msgs::msg::Marker delete_marker;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    marker_array.markers.push_back(delete_marker);

    for (const auto & det : tracked_dets.detections) {
      visualization_msgs::msg::Marker marker;
      marker.header = tracked_dets.header;
      marker.ns = "tracked_objects";
      marker.id = std::stoi(det.id);
      marker.type = visualization_msgs::msg::Marker::CUBE;
      marker.action = visualization_msgs::msg::Marker::ADD;

      marker.pose.position = det.bbox.center.position;
      marker.pose.orientation = det.bbox.center.orientation;
      marker.scale.x = std::max(0.1, det.bbox.size.x);
      marker.scale.y = std::max(0.1, det.bbox.size.y);
      marker.scale.z = std::max(0.1, det.bbox.size.z);

      // Color from attribute hypotheses, falling back to deterministic track ID color
      int trk_id = std::stoi(det.id);
      marker.color.r = static_cast<float>((trk_id * 67) % 255) / 255.0f;
      marker.color.g = static_cast<float>((trk_id * 123) % 255) / 255.0f;
      marker.color.b = static_cast<float>((trk_id * 200) % 255) / 255.0f;
      marker.color.a = 0.6f;

      // Find best attribute hypothesis to determine marker color (has to be greater than 50%)
      double best_attr_score = 0.5;
      std::string best_attr;
      for (const auto & result : det.results) {
        const auto & cid = result.hypothesis.class_id;
        if (cid.find(':') != std::string::npos && result.hypothesis.score > best_attr_score) {
          best_attr_score = result.hypothesis.score;
          best_attr = cid;
        }
      }

      if (!best_attr.empty()) {
        if (best_attr == "state:red") {
          marker.color.r = 1.0f;
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
        } else if (best_attr == "state:yellow") {
          marker.color.r = 1.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
        } else if (best_attr == "state:green") {
          marker.color.r = 0.0f;
          marker.color.g = 1.0f;
          marker.color.b = 0.0f;
        } else if (best_attr == "behavior:braking") {
          marker.color.r = 1.0f;
          marker.color.g = 0.0f;
          marker.color.b = 0.0f;
        } else if (best_attr == "behavior:turning_left" || best_attr == "behavior:turning_right") {
          marker.color.r = 1.0f;
          marker.color.g = 0.7f;
          marker.color.b = 0.0f;
        } else if (best_attr == "behavior:hazard_lights") {
          marker.color.r = 1.0f;
          marker.color.g = 0.5f;
          marker.color.b = 0.0f;
        }
        marker.color.a = 0.8f;
      }
      marker.lifetime = rclcpp::Duration::from_seconds(0.5);

      marker_array.markers.push_back(marker);

      // Text marker for track ID and class
      visualization_msgs::msg::Marker text_marker;
      text_marker.header = tracked_dets.header;
      text_marker.ns = "tracked_labels";
      text_marker.id = trk_id;
      text_marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text_marker.action = visualization_msgs::msg::Marker::ADD;

      text_marker.pose.position = det.bbox.center.position;
      text_marker.pose.position.z += det.bbox.size.z * 0.5 + 0.3;
      text_marker.scale.z = 0.5;

      std::string class_name = det.results.empty() ? "?" : det.results[0].hypothesis.class_id;
      std::string label = class_name + " #" + det.id;
      if (!best_attr.empty()) {
        // Show attribute after the colon (e.g. "car #5 [braking]")
        label += " [" + best_attr.substr(best_attr.find(':') + 1) + "]";
      }
      text_marker.text = label;

      text_marker.color.r = 1.0f;
      text_marker.color.g = 1.0f;
      text_marker.color.b = 1.0f;
      text_marker.color.a = 1.0f;
      text_marker.lifetime = rclcpp::Duration::from_seconds(0.5);

      marker_array.markers.push_back(text_marker);
    }

    markers_pub_->publish(marker_array);
  }  // publish_visualization_

  RCLCPP_DEBUG(
    this->get_logger(),
    "First track - Class: '%s'; ID: %d",
    tracked_dets.detections[0].results[0].hypothesis.class_id.c_str(),
    std::stoi(tracked_dets.detections[0].id));
}

RCLCPP_COMPONENTS_REGISTER_NODE(TrackingNode)
