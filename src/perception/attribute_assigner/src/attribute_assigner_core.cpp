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

#include "attribute_assigner/attribute_assigner_core.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace wato::perception::attribute_assigner
{

AttributeAssignerCore::AttributeAssignerCore(const Params & params)
: params_(params)
, traffic_light_ids_(params.traffic_light_class_ids.begin(), params.traffic_light_class_ids.end())
, car_ids_(params.car_class_ids.begin(), params.car_class_ids.end())
, truck_ids_(params.truck_class_ids.begin(), params.truck_class_ids.end())
, bus_ids_(params.bus_class_ids.begin(), params.bus_class_ids.end())
{}

vision_msgs::msg::Detection2DArray AttributeAssignerCore::process(
  const cv::Mat & image, const vision_msgs::msg::Detection2DArray & input)
{
  const auto start = std::chrono::steady_clock::now();

  vision_msgs::msg::Detection2DArray output;
  output.header = input.header;
  output.detections.reserve(input.detections.size());

  if (image.empty()) {
    // No image: pass through detections unchanged
    output.detections = input.detections;
    last_processing_time_ms_ = 0.0;
    processed_count_ += input.detections.size();
    return output;
  }

  for (const auto & det : input.detections) {
    vision_msgs::msg::Detection2D enriched = det;

    const double score = getBestScore(det);
    const bool is_traffic_light = isTrafficLight(det);
    const bool is_car = isCar(det);

    // Skip cropping entirely for classes we don't process
    if (!is_traffic_light && !is_car) {
      output.detections.push_back(enriched);
      continue;
    }

    if (score < params_.min_detection_confidence) {
      output.detections.push_back(enriched);
      continue;
    }

    cv::Mat crop = cropToBbox(image, det);
    if (crop.empty()) {
      output.detections.push_back(enriched);
      continue;
    }

    if (is_traffic_light) {
      TrafficLightAttributes attrs = classifyTrafficLightState(crop);
      appendTrafficLightHypotheses(enriched, attrs);
    } else {
      CarAttributes attrs = classifyCarBehavior(crop);
      appendCarHypotheses(enriched, attrs);
    }

    output.detections.push_back(enriched);
  }

  const auto end = std::chrono::steady_clock::now();
  last_processing_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
  processed_count_ += input.detections.size();

  return output;
}

uint64_t AttributeAssignerCore::getProcessedCount() const
{
  return processed_count_;
}

double AttributeAssignerCore::getLastProcessingTimeMs() const
{
  return last_processing_time_ms_;
}

bool AttributeAssignerCore::isTrafficLight(const vision_msgs::msg::Detection2D & det) const
{
  for (const auto & result : det.results) {
    if (traffic_light_ids_.count(result.hypothesis.class_id) > 0) {
      return true;
    }
  }
  return false;
}

bool AttributeAssignerCore::isCar(const vision_msgs::msg::Detection2D & det) const
{
  for (const auto & result : det.results) {
    if (car_ids_.count(result.hypothesis.class_id) > 0) {
      return true;
    }
  }
  return false;
}

bool AttributeAssignerCore::isTrafficLightClassId(const std::string & class_id) const
{
  return traffic_light_ids_.count(class_id) > 0;
}

bool AttributeAssignerCore::isCarClassId(const std::string & class_id) const
{
  return car_ids_.count(class_id) > 0;
}

bool AttributeAssignerCore::isTruckClassId(const std::string & class_id) const
{
  return truck_ids_.count(class_id) > 0;
}

bool AttributeAssignerCore::isBusClassId(const std::string & class_id) const
{
  return bus_ids_.count(class_id) > 0;
}

std::string AttributeAssignerCore::getBestClassId(const vision_msgs::msg::Detection2D & det)
{
  if (det.results.empty()) {
    return "";
  }

  const auto best = std::max_element(
    det.results.begin(),
    det.results.end(),
    [](const vision_msgs::msg::ObjectHypothesisWithPose & a, const vision_msgs::msg::ObjectHypothesisWithPose & b) {
      return a.hypothesis.score < b.hypothesis.score;
    });

  return best->hypothesis.class_id;
}

double AttributeAssignerCore::getBestScore(const vision_msgs::msg::Detection2D & det)
{
  if (det.results.empty()) {
    return 0.0;
  }

  const auto best = std::max_element(
    det.results.begin(),
    det.results.end(),
    [](const vision_msgs::msg::ObjectHypothesisWithPose & a, const vision_msgs::msg::ObjectHypothesisWithPose & b) {
      return a.hypothesis.score < b.hypothesis.score;
    });

  return best->hypothesis.score;
}

cv::Mat AttributeAssignerCore::cropToBbox(const cv::Mat & image, const vision_msgs::msg::Detection2D & det) const
{
  const double cx = det.bbox.center.position.x;
  const double cy = det.bbox.center.position.y;
  const double w = det.bbox.size_x;
  const double h = det.bbox.size_y;

  if (w <= 0 || h <= 0) {
    return cv::Mat();
  }

  const int x1 = static_cast<int>(std::round(cx - w / 2.0));
  const int y1 = static_cast<int>(std::round(cy - h / 2.0));
  const int x2 = static_cast<int>(std::round(cx + w / 2.0));
  const int y2 = static_cast<int>(std::round(cy + h / 2.0));

  const int img_w = image.cols;
  const int img_h = image.rows;

  const int x1_c = std::max(0, std::min(x1, img_w - 1));
  const int y1_c = std::max(0, std::min(y1, img_h - 1));
  const int x2_c = std::max(0, std::min(x2, img_w));
  const int y2_c = std::max(0, std::min(y2, img_h));

  if (x1_c >= x2_c || y1_c >= y2_c) {
    return cv::Mat();
  }

  // Return a view (no copy); valid only while image is unchanged. Caller must use synchronously.
  return image(cv::Rect(x1_c, y1_c, x2_c - x1_c, y2_c - y1_c));
}

TrafficLightAttributes AttributeAssignerCore::classifyTrafficLightState(const cv::Mat & crop) const
{
  // Safety-first prior — default to red when there's insufficient data to classify
  TrafficLightAttributes attrs;
  attrs.red = 1.0;
  attrs.green = 0.0;
  attrs.yellow = 0.0;

  if (crop.rows < 3 || crop.cols < 2) {
    return attrs;
  }

  // Convert to HSV once
  cv::Mat hsv;
  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

  // Determine orientation: vertical (tall) or horizontal (wide) traffic light
  const bool is_horizontal = crop.cols > crop.rows;

  // Compute the center strip bounds along the shorter axis.
  // For vertical TLs: strip columns
  // For horizontal TLs: strip rows
  int row_start = 0, row_end = hsv.rows;
  int col_start = 0, col_end = hsv.cols;

  const double margin_frac = std::clamp(params_.traffic_light_strip_margin, 0.0, 0.45);

  if (is_horizontal) {
    const int margin = static_cast<int>(hsv.rows * margin_frac);
    row_start = margin;
    row_end = hsv.rows - margin;
  } else {
    const int margin = static_cast<int>(hsv.cols * margin_frac);
    col_start = margin;
    col_end = hsv.cols - margin;
  }

  // Clamp to valid range
  row_start = std::max(row_start, 0);
  row_end = std::max(row_end, row_start + 1);
  col_start = std::max(col_start, 0);
  col_end = std::max(col_end, col_start + 1);

  // Cache param thresholds as uint8 to avoid repeated casts in the hot loop
  const uint8_t min_val = static_cast<uint8_t>(params_.traffic_light_min_value);
  const uint8_t min_sat = static_cast<uint8_t>(params_.traffic_light_min_saturation);
  const uint8_t red_lo = static_cast<uint8_t>(params_.traffic_light_red_hue_lo);
  const uint8_t red_hi = static_cast<uint8_t>(params_.traffic_light_red_hue_hi);
  const uint8_t yellow_hi = static_cast<uint8_t>(params_.traffic_light_yellow_hue_hi);
  const uint8_t green_hi = static_cast<uint8_t>(params_.traffic_light_green_hue_hi);

  // Single pass over center strip pixels: count bright pixels per hue bucket
  // HSV is interleaved as [H,S,V, H,S,V, ...] — no need to cv::split
  int red_count = 0;
  int yellow_count = 0;
  int green_count = 0;
  int bright_count = 0;

  for (int r = row_start; r < row_end; ++r) {
    const uint8_t * row = hsv.ptr<uint8_t>(r);
    for (int c = col_start; c < col_end; ++c) {
      const uint8_t s = row[c * 3 + 1];
      const uint8_t v = row[c * 3 + 2];
      if (v < min_val || s < min_sat) continue;

      ++bright_count;
      const uint8_t h = row[c * 3];

      if (h <= red_lo || h >= red_hi) {
        ++red_count;
      } else if (h <= yellow_hi) {
        ++yellow_count;
      } else if (h < green_hi) {
        ++green_count;
      }
      // Pixels with hue outside all ranges (blue/purple) are bright but unclassified
    }
  }

  const int total_pixels = (row_end - row_start) * (col_end - col_start);

  // Classified = pixels that fell into a known hue bucket
  const int classified = red_count + yellow_count + green_count;

  // If too few bright pixels or none classified, default to red (safety-first for AV)
  if (bright_count < total_pixels * 0.01 || bright_count < 4 || classified == 0) {
    attrs.red = 1.0;
    attrs.green = 0.0;
    attrs.yellow = 0.0;
    return attrs;
  }

  // Confidence is the actual pixel ratio — how dominant is each color among classified pixels
  const double inv = 1.0 / static_cast<double>(classified);
  attrs.red = red_count * inv;
  attrs.green = green_count * inv;
  attrs.yellow = yellow_count * inv;

  return attrs;
}

CarAttributes AttributeAssignerCore::classifyCarBehavior(const cv::Mat & crop) const
{
  CarAttributes attrs;  // all zero by default

  if (crop.rows < 10 || crop.cols < 10) {
    return attrs;
  }

  const int rows = crop.rows;
  const int cols = crop.cols;
  const double center_x = cols / 2.0;
  const double bbox_area = static_cast<double>(rows * cols);

  // Cache params
  const uint8_t red_lo = static_cast<uint8_t>(params_.car_red_hue_lo);
  const uint8_t red_hi = static_cast<uint8_t>(params_.car_red_hue_hi);
  const uint8_t amber_lo = static_cast<uint8_t>(params_.car_amber_hue_lo);
  const uint8_t amber_hi = static_cast<uint8_t>(params_.car_amber_hue_hi);
  const uint8_t min_brake_v = static_cast<uint8_t>(params_.car_brake_min_brightness);
  const uint8_t min_brake_s = static_cast<uint8_t>(params_.car_brake_min_saturation);
  const uint8_t min_amber_s = static_cast<uint8_t>(params_.car_amber_min_saturation);
  const uint8_t min_amber_v = static_cast<uint8_t>(params_.car_amber_min_value);

  // Convert to HSV once
  cv::Mat hsv;
  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

  // ============================================================
  // Single-pass pixel scan: build red and amber masks in one loop
  // Avoids cv::split + cv::inRange + cv::threshold + cv::bitwise_and chain
  // ============================================================
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

      // Red: wrapping hue, with saturation and brightness thresholds
      if ((h <= red_lo || h >= red_hi) && s >= min_brake_s && v >= min_brake_v) {
        red_row[c] = 255;
      }
      // Amber: hue in range, with saturation and brightness thresholds
      if (h >= amber_lo && h <= amber_hi && s >= min_amber_s && v >= min_amber_v) {
        amber_row[c] = 255;
      }
    }
  }

  // ============================================================
  // Brake light detection via red blobs
  // ============================================================
  const double min_blob_area = bbox_area * 0.001;  // 0.1% — brake lights are small at distance
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

  // Red blobs in lower 70% of bbox (brake lights sit low on the rear)
  auto red_blobs = extractBlobs(red_mask, 0.3);

  if (red_blobs.size() >= 2) {
    // Find best symmetric pair — geometry is the primary signal
    double best_pair_score = 0.0;
    for (size_t i = 0; i < red_blobs.size(); ++i) {
      for (size_t j = i + 1; j < red_blobs.size(); ++j) {
        const auto & b1 = red_blobs[i];
        const auto & b2 = red_blobs[j];

        // Horizontal alignment (similar Y)
        double y_alignment = 1.0 - std::min(1.0, std::abs(b1.cy - b2.cy) / (rows * 0.2));

        // Symmetry about center
        double symmetry =
          std::max(0.0, 1.0 - std::abs(std::abs(b1.cx - center_x) - std::abs(b2.cx - center_x)) / (cols * 0.5));

        // Separation (not the same blob split apart)
        double sep = std::min(1.0, std::abs(b1.cx - b2.cx) / (cols * 0.2));

        double geometry = y_alignment * 0.4 + symmetry * 0.4 + sep * 0.2;
        best_pair_score = std::max(best_pair_score, geometry);
      }
    }
    // Two red blobs that passed size/position filters is already strong evidence.
    // Geometry score (0-1) maps to confidence range [0.4, 0.95].
    attrs.braking = std::min(0.95, 0.4 + best_pair_score * 0.55);
  } else if (red_blobs.size() == 1) {
    // Single red blob — moderate confidence based on relative size
    double size_ratio = red_blobs[0].area / bbox_area;
    attrs.braking = std::min(0.6, 0.2 + std::min(1.0, size_ratio / 0.02) * 0.4);
  }

  // ============================================================
  // Turn signal / hazard detection via amber blobs
  // ============================================================
  auto amber_blobs = extractBlobs(amber_mask, 0.0);

  if (!amber_blobs.empty()) {
    // Accumulate per-side amber area (not just blob count)
    double left_area = 0.0, right_area = 0.0;

    for (const auto & blob : amber_blobs) {
      double normalized_x = (blob.cx - center_x) / (cols * 0.5);
      if (normalized_x < -0.05) {
        left_area += blob.area;
      } else if (normalized_x > 0.05) {
        right_area += blob.area;
      } else {
        // Ambiguous center blob — split evenly
        left_area += blob.area * 0.5;
        right_area += blob.area * 0.5;
      }
    }

    // Confidence is amber area as fraction of bbox, scaled to [0, ~0.95]
    // A strong turn signal covers ~1-3% of bbox
    double left_conf = std::min(0.95, left_area / (bbox_area * 0.015));
    double right_conf = std::min(0.95, right_area / (bbox_area * 0.015));

    // Hazard: both sides active — confidence is the weaker side (bottleneck)
    if (left_conf > 0.1 && right_conf > 0.1) {
      attrs.hazard_lights = std::min(left_conf, right_conf);
      // Downweight individual turn signals when hazard is likely
      attrs.turning_left = left_conf * 0.3;
      attrs.turning_right = right_conf * 0.3;
    } else {
      attrs.turning_left = left_conf;
      attrs.turning_right = right_conf;
    }
  }

  return attrs;
}

void AttributeAssignerCore::appendTrafficLightHypotheses(
  vision_msgs::msg::Detection2D & det, const TrafficLightAttributes & attrs)
{
  det.results.push_back(makeHypothesis(std::string(kStatePrefix) + "green", attrs.green));
  det.results.push_back(makeHypothesis(std::string(kStatePrefix) + "yellow", attrs.yellow));
  det.results.push_back(makeHypothesis(std::string(kStatePrefix) + "red", attrs.red));
}

void AttributeAssignerCore::appendCarHypotheses(vision_msgs::msg::Detection2D & det, const CarAttributes & attrs)
{
  det.results.push_back(makeHypothesis(std::string(kBehaviorPrefix) + "turning_left", attrs.turning_left));
  det.results.push_back(makeHypothesis(std::string(kBehaviorPrefix) + "turning_right", attrs.turning_right));
  det.results.push_back(makeHypothesis(std::string(kBehaviorPrefix) + "braking", attrs.braking));
  det.results.push_back(makeHypothesis(std::string(kBehaviorPrefix) + "hazard_lights", attrs.hazard_lights));
}

vision_msgs::msg::ObjectHypothesisWithPose AttributeAssignerCore::makeHypothesis(
  const std::string & class_id, double score)
{
  vision_msgs::msg::ObjectHypothesisWithPose hyp;
  hyp.hypothesis.class_id = class_id;
  hyp.hypothesis.score = score;
  return hyp;
}

}  // namespace wato::perception::attribute_assigner
