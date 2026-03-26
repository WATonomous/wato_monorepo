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

#include "attribute_assigner/car_behavior_classifier.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/imgproc.hpp>

namespace wato::perception::attribute_assigner
{

CarBehaviorClassifier::CarBehaviorClassifier(const Params & params)
: params_(params)
, truck_ids_(params.truck_class_ids.begin(), params.truck_class_ids.end())
, bus_ids_(params.bus_class_ids.begin(), params.bus_class_ids.end())
{
  // Build match set as union of car + truck + bus class IDs
  ids_.insert(params.class_ids.begin(), params.class_ids.end());
  ids_.insert(params.truck_class_ids.begin(), params.truck_class_ids.end());
  ids_.insert(params.bus_class_ids.begin(), params.bus_class_ids.end());
}

bool CarBehaviorClassifier::matches(const vision_msgs::msg::Detection2D & det) const
{
  for (const auto & result : det.results) {
    if (ids_.count(result.hypothesis.class_id) > 0) {
      return true;
    }
  }
  return false;
}

bool CarBehaviorClassifier::matchesClassId(const std::string & class_id) const
{
  return ids_.count(class_id) > 0;
}

void CarBehaviorClassifier::classify(const cv::Mat & crop, vision_msgs::msg::Detection2D & det) const
{
  double braking = 0.0, turning_left = 0.0, turning_right = 0.0, hazard_lights = 0.0;

  if (crop.rows >= 10 && crop.cols >= 10) {
    const int rows = crop.rows;
    const int cols = crop.cols;
    const double center_x = cols / 2.0;
    const double bbox_area = static_cast<double>(rows * cols);

    // Cache params
    const uint8_t red_lo = static_cast<uint8_t>(params_.red_hue_lo);
    const uint8_t red_hi = static_cast<uint8_t>(params_.red_hue_hi);
    const uint8_t amber_lo = static_cast<uint8_t>(params_.amber_hue_lo);
    const uint8_t amber_hi = static_cast<uint8_t>(params_.amber_hue_hi);
    const uint8_t min_brake_v = static_cast<uint8_t>(params_.brake_min_brightness);
    const uint8_t min_brake_s = static_cast<uint8_t>(params_.brake_min_saturation);
    const uint8_t min_amber_s = static_cast<uint8_t>(params_.amber_min_saturation);
    const uint8_t min_amber_v = static_cast<uint8_t>(params_.amber_min_value);

    // Convert to HSV once
    cv::Mat hsv;
    cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

    // Single-pass pixel scan: build red and amber masks in one loop
    cv::Mat red_mask(rows, cols, CV_8UC1, cv::Scalar(0));
    cv::Mat amber_mask(rows, cols, CV_8UC1, cv::Scalar(0));

    for (int r = 0; r < rows; ++r) {
      const uint8_t * hsv_row = hsv.ptr<uint8_t>(r);
      uint8_t * red_row = red_mask.ptr<uint8_t>(r);
      uint8_t * amber_row = amber_mask.ptr<uint8_t>(r);
      for (int c = 0; c < cols; ++c) {
        const uint8_t h = hsv_row[c * 3];
        const uint8_t s = hsv_row[c * 3 + 1];
        const uint8_t v = hsv_row[c * 3 + 2];

        if ((h <= red_lo || h >= red_hi) && s >= min_brake_s && v >= min_brake_v) {
          red_row[c] = 255;
        }
        if (h >= amber_lo && h <= amber_hi && s >= min_amber_s && v >= min_amber_v) {
          amber_row[c] = 255;
        }
      }
    }

    // Blob extraction helper
    const double min_blob_area = bbox_area * 0.001;
    const double max_blob_area = bbox_area * 0.35;

    struct Blob
    {
      double cx, cy;
      int area;
    };

    auto extractBlobs = [&](const cv::Mat & mask, double min_cy_ratio) {
      std::vector<Blob> blobs;
      cv::Mat labels, stats, centroids;
      int n = cv::connectedComponentsWithStats(mask, labels, stats, centroids);
      for (int i = 1; i < n; ++i) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area >= min_blob_area && area <= max_blob_area) {
          double cy = centroids.at<double>(i, 1);
          if (cy > rows * min_cy_ratio) {
            blobs.push_back({centroids.at<double>(i, 0), cy, area});
          }
        }
      }
      return blobs;
    };

    // Brake light detection via red blobs in lower 70% of bbox
    auto red_blobs = extractBlobs(red_mask, 0.3);

    if (red_blobs.size() >= 2) {
      double best_pair_score = 0.0;
      for (size_t i = 0; i < red_blobs.size(); ++i) {
        for (size_t j = i + 1; j < red_blobs.size(); ++j) {
          const auto & b1 = red_blobs[i];
          const auto & b2 = red_blobs[j];

          double y_alignment = 1.0 - std::min(1.0, std::abs(b1.cy - b2.cy) / (rows * 0.2));
          double symmetry =
            std::max(0.0, 1.0 - std::abs(std::abs(b1.cx - center_x) - std::abs(b2.cx - center_x)) / (cols * 0.5));
          double sep = std::min(1.0, std::abs(b1.cx - b2.cx) / (cols * 0.2));
          double geometry = y_alignment * 0.4 + symmetry * 0.4 + sep * 0.2;
          best_pair_score = std::max(best_pair_score, geometry);
        }
      }
      braking = std::min(0.95, 0.4 + best_pair_score * 0.55);
    } else if (red_blobs.size() == 1) {
      double size_ratio = red_blobs[0].area / bbox_area;
      braking = std::min(0.6, 0.2 + std::min(1.0, size_ratio / 0.02) * 0.4);
    }

    // Turn signal / hazard detection via amber blobs
    auto amber_blobs = extractBlobs(amber_mask, 0.0);

    if (!amber_blobs.empty()) {
      double left_area = 0.0, right_area = 0.0;
      for (const auto & blob : amber_blobs) {
        double normalized_x = (blob.cx - center_x) / (cols * 0.5);
        if (normalized_x < -0.05) {
          left_area += blob.area;
        } else if (normalized_x > 0.05) {
          right_area += blob.area;
        } else {
          left_area += blob.area * 0.5;
          right_area += blob.area * 0.5;
        }
      }

      double left_conf = std::min(0.95, left_area / (bbox_area * 0.015));
      double right_conf = std::min(0.95, right_area / (bbox_area * 0.015));

      if (left_conf > 0.1 && right_conf > 0.1) {
        hazard_lights = std::min(left_conf, right_conf);
        turning_left = left_conf * 0.3;
        turning_right = right_conf * 0.3;
      } else {
        turning_left = left_conf;
        turning_right = right_conf;
      }
    }
  }

  det.results.push_back(makeHypothesis(std::string(kPrefix) + "turning_left", turning_left));
  det.results.push_back(makeHypothesis(std::string(kPrefix) + "turning_right", turning_right));
  det.results.push_back(makeHypothesis(std::string(kPrefix) + "braking", braking));
  det.results.push_back(makeHypothesis(std::string(kPrefix) + "hazard_lights", hazard_lights));
}

BoxParams3D CarBehaviorClassifier::get3DBoxParams(const vision_msgs::msg::Detection2D & det) const
{
  // Determine vehicle subtype from detection hypotheses
  bool is_truck = false;
  bool is_bus = false;
  for (const auto & result : det.results) {
    if (truck_ids_.count(result.hypothesis.class_id) > 0) {
      is_truck = true;
      break;
    }
    if (bus_ids_.count(result.hypothesis.class_id) > 0) {
      is_bus = true;
      break;
    }
  }

  BoxParams3D box;
  box.assumed_depth = params_.assumed_depth;
  // Use height for depth estimation — car height is visible from any angle
  // (front, rear, or side), unlike width which changes drastically for side views
  box.use_width_for_depth = false;

  if (is_bus) {
    box.width = params_.bus_width;
    box.height = params_.bus_height;
    box.length = params_.bus_length;
  } else if (is_truck) {
    box.width = params_.truck_width;
    box.height = params_.truck_height;
    box.length = params_.truck_length;
  } else {
    box.width = params_.car_width;
    box.height = params_.car_height;
    box.length = params_.car_length;
  }

  return box;
}

MarkerColor CarBehaviorClassifier::getMarkerColor(const vision_msgs::msg::Detection2D & det) const
{
  // Default: cyan
  MarkerColor color;
  color.r = 0.0f;
  color.g = 0.8f;
  color.b = 1.0f;
  color.a = 0.4f;

  double best_score = 0.0;
  std::string best_behavior;
  for (const auto & result : det.results) {
    const std::string & cid = result.hypothesis.class_id;
    if (
      (cid == "behavior:braking" || cid == "behavior:turning_left" || cid == "behavior:turning_right" ||
       cid == "behavior:hazard_lights") &&
      result.hypothesis.score > best_score)
    {
      best_score = result.hypothesis.score;
      best_behavior = cid;
    }
  }

  if (best_score > 0.15) {
    if (best_behavior == "behavior:braking") {
      color = {1.0f, 0.0f, 0.0f, 0.6f};
    } else if (best_behavior == "behavior:turning_left" || best_behavior == "behavior:turning_right") {
      color = {1.0f, 1.0f, 0.0f, 0.6f};
    } else if (best_behavior == "behavior:hazard_lights") {
      color = {1.0f, 0.65f, 0.0f, 0.6f};
    }
  }

  return color;
}

}  // namespace wato::perception::attribute_assigner
