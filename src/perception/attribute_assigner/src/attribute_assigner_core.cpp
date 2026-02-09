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

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace
{

/**
 * @brief Compute mean of a rectangular region from an integral image in O(1).
 * @param integral Integral image (rows+1)x(cols+1), 3-channel CV_64F (e.g. from cv::integral(hsv)).
 * @param x Left column of region
 * @param y Top row of region
 * @param w Width of region
 * @param h Height of region
 * @return Mean as cv::Scalar (3 values: channel 0, 1, 2)
 */
cv::Scalar regionMeanFromIntegral(const cv::Mat & integral, int x, int y, int w, int h)
{
  const int rows = integral.rows - 1;
  const int cols = integral.cols - 1;
  const int x2 = std::min(x + w, cols);
  const int y2 = std::min(y + h, rows);
  const int x1 = std::max(0, x);
  const int y1 = std::max(0, y);
  const double area = static_cast<double>(x2 - x1) * (y2 - y1);
  if (area <= 0.0) {
    return cv::Scalar(0.0, 0.0, 0.0);
  }

  const cv::Vec3d & a = integral.at<cv::Vec3d>(y1, x1);
  const cv::Vec3d & b = integral.at<cv::Vec3d>(y1, x2);
  const cv::Vec3d & c = integral.at<cv::Vec3d>(y2, x1);
  const cv::Vec3d & d = integral.at<cv::Vec3d>(y2, x2);
  return cv::Scalar(
    (d[0] - b[0] - c[0] + a[0]) / area, (d[1] - b[1] - c[1] + a[1]) / area, (d[2] - b[2] - c[2] + a[2]) / area);
}

}  // namespace

namespace wato::perception::attribute_assigner
{

AttributeAssignerCore::AttributeAssignerCore(const Params & params)
: params_(params)
, traffic_light_ids_(params.traffic_light_class_ids.begin(), params.traffic_light_class_ids.end())
, car_ids_(params.car_class_ids.begin(), params.car_class_ids.end())
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
  const std::string class_id = getBestClassId(det);
  return traffic_light_ids_.count(class_id) > 0;
}

bool AttributeAssignerCore::isCar(const vision_msgs::msg::Detection2D & det) const
{
  const std::string class_id = getBestClassId(det);
  return car_ids_.count(class_id) > 0;
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
  TrafficLightAttributes attrs;
  const double min_sat = params_.traffic_light_min_saturation;
  const double min_val = params_.traffic_light_min_value;

  if (crop.rows < 3 || crop.cols < 2) {
    attrs.red = 0.33;
    attrs.green = 0.33;
    attrs.yellow = 0.33;
    return attrs;
  }

  cv::Mat hsv;
  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);
  cv::Mat sum_integral;
  cv::integral(hsv, sum_integral, CV_64F);

  const int crop_w = crop.cols;
  const int crop_h = crop.rows;
  const double aspect = static_cast<double>(crop_h) / static_cast<double>(crop_w);

  // Vertical light (standard 3-stack): top = red, middle = yellow, bottom = green (O(1) means via integral)
  if (aspect > 1.2) {
    const int h = crop_h;
    const int third = h / 3;
    cv::Scalar mean_top = regionMeanFromIntegral(sum_integral, 0, 0, crop_w, third);
    cv::Scalar mean_mid = regionMeanFromIntegral(sum_integral, 0, third, crop_w, third);
    cv::Scalar mean_bot = regionMeanFromIntegral(sum_integral, 0, 2 * third, crop_w, h - 2 * third);

    auto is_lit = [min_sat, min_val](const cv::Scalar & m) { return m[1] >= min_sat && m[2] >= min_val; };
    auto hue_red = [](double h) { return h <= 10.0 || h >= 170.0; };
    auto hue_yellow = [](double h) { return h > 10.0 && h <= 35.0; };
    auto hue_green = [](double h) { return h > 35.0 && h < 85.0; };

    const double h_top = mean_top[0], s_top = mean_top[1];
    const double s_mid = mean_mid[1];
    const double h_bot = mean_bot[0], s_bot = mean_bot[1];

    const bool lit_top = is_lit(mean_top);
    const bool lit_mid = is_lit(mean_mid);
    const bool lit_bot = is_lit(mean_bot);

    // One region lit -> assign that color
    if (lit_top && !lit_mid && !lit_bot) {
      if (hue_red(h_top)) {
        attrs.red = 0.92;
        attrs.green = 0.05;
        attrs.yellow = 0.05;
      } else if (hue_yellow(h_top)) {
        attrs.yellow = 0.92;
        attrs.red = 0.05;
        attrs.green = 0.05;
      } else if (hue_green(h_top)) {
        attrs.green = 0.92;
        attrs.red = 0.05;
        attrs.yellow = 0.05;
      } else {
        attrs.red = 0.4;
        attrs.green = 0.3;
        attrs.yellow = 0.3;
      }
      return attrs;
    }
    if (!lit_top && lit_mid && !lit_bot) {
      attrs.yellow = 0.92;
      attrs.red = 0.05;
      attrs.green = 0.05;
      return attrs;
    }
    if (!lit_top && !lit_mid && lit_bot) {
      if (hue_green(h_bot)) {
        attrs.green = 0.92;
        attrs.red = 0.05;
        attrs.yellow = 0.05;
      } else if (hue_yellow(h_bot)) {
        attrs.yellow = 0.92;
        attrs.red = 0.05;
        attrs.green = 0.05;
      } else {
        attrs.green = 0.5;
        attrs.yellow = 0.3;
        attrs.red = 0.2;
      }
      return attrs;
    }

    // Multiple lit: pick strongest by saturation
    if (lit_top || lit_mid || lit_bot) {
      double best_s = 0.0;
      int best = -1;  // 0=red, 1=yellow, 2=green
      if (lit_top && s_top > best_s) {
        best_s = s_top;
        best = 0;
      }
      if (lit_mid && s_mid > best_s) {
        best_s = s_mid;
        best = 1;
      }
      if (lit_bot && s_bot > best_s) {
        best_s = s_bot;
        best = 2;
      }
      if (best == 0) {
        attrs.red = 0.85;
        attrs.yellow = 0.08;
        attrs.green = 0.07;
      } else if (best == 1) {
        attrs.yellow = 0.85;
        attrs.red = 0.08;
        attrs.green = 0.07;
      } else if (best == 2) {
        attrs.green = 0.85;
        attrs.red = 0.05;
        attrs.yellow = 0.10;
      }
      return attrs;
    }
  }

  // Horizontal or roughly square: single region
  if (aspect <= 1.2) {
    cv::Scalar mean_all = regionMeanFromIntegral(sum_integral, 0, 0, crop_w, crop_h);
    const double h = mean_all[0], s = mean_all[1], v = mean_all[2];
    if (s >= min_sat && v >= min_val) {
      if (h <= 10.0 || h >= 170.0) {
        attrs.red = 0.88;
        attrs.green = 0.06;
        attrs.yellow = 0.06;
      } else if (h > 10.0 && h <= 35.0) {
        attrs.yellow = 0.88;
        attrs.red = 0.06;
        attrs.green = 0.06;
      } else if (h > 35.0 && h < 85.0) {
        attrs.green = 0.88;
        attrs.red = 0.06;
        attrs.yellow = 0.06;
      } else {
        attrs.red = 0.33;
        attrs.green = 0.33;
        attrs.yellow = 0.33;
      }
      return attrs;
    }
  }

  // No clear lit region
  attrs.red = 0.33;
  attrs.green = 0.33;
  attrs.yellow = 0.33;
  return attrs;
}

CarAttributes AttributeAssignerCore::classifyCarBehavior(const cv::Mat & crop) const
{
  CarAttributes attrs;
  if (crop.rows < 4 || crop.cols < 4) {
    attrs.turning_left = 0.1;
    attrs.turning_right = 0.1;
    attrs.braking = 0.1;
    attrs.hazard_lights = 0.05;
    return attrs;
  }

  const int h = crop.rows;
  const int w = crop.cols;
  const double min_bright = params_.car_brake_min_brightness;  // V in HSV
  const double ah_lo = params_.car_amber_hue_lo;
  const double ah_hi = params_.car_amber_hue_hi;
  const double am_sat = params_.car_amber_min_saturation;
  const double am_val = params_.car_amber_min_value;

  // Single BGR→HSV conversion for whole crop; integral image for O(1) region means
  cv::Mat crop_hsv;
  cv::cvtColor(crop, crop_hsv, cv::COLOR_BGR2HSV);
  cv::Mat sum_integral;
  cv::integral(crop_hsv, sum_integral, CV_64F);

  // Bottom 25%: brake lights (red in HSV: H in [0,10] or [170,180], high S and V)
  const int bottom_h = std::max(1, h / 4);
  cv::Scalar mean_bottom = regionMeanFromIntegral(sum_integral, 0, h - bottom_h, w, bottom_h);
  const double bh = mean_bottom[0], bs = mean_bottom[1], bv = mean_bottom[2];
  const bool is_red = (bh <= 10.0 || bh >= 170.0) && bs >= 80.0 && bv >= min_bright;
  attrs.braking = is_red ? 0.85 : 0.1;

  // Left 25% and right 25%: amber for turn/hazard
  const int side_w = std::max(1, w / 4);
  cv::Scalar mean_left = regionMeanFromIntegral(sum_integral, 0, 0, side_w, h);
  cv::Scalar mean_right = regionMeanFromIntegral(sum_integral, w - side_w, 0, side_w, h);

  auto has_amber = [ah_lo, ah_hi, am_sat, am_val](const cv::Scalar & m) {
    const double h = m[0], s = m[1], v = m[2];
    return (h >= ah_lo && h <= ah_hi) && s >= am_sat && v >= am_val;
  };

  const bool left_amber = has_amber(mean_left);
  const bool right_amber = has_amber(mean_right);

  if (left_amber && right_amber) {
    attrs.hazard_lights = 0.82;
    attrs.turning_left = 0.1;
    attrs.turning_right = 0.1;
  } else if (left_amber && !right_amber) {
    attrs.turning_left = 0.82;
    attrs.turning_right = 0.08;
    attrs.hazard_lights = 0.05;
  } else if (!left_amber && right_amber) {
    attrs.turning_right = 0.82;
    attrs.turning_left = 0.08;
    attrs.hazard_lights = 0.05;
  } else {
    attrs.turning_left = 0.08;
    attrs.turning_right = 0.08;
    attrs.hazard_lights = 0.03;
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
