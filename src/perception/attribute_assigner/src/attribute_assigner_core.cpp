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

#include <opencv2/imgproc.hpp>

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
    if (score < params_.min_detection_confidence) {
      output.detections.push_back(enriched);
      continue;
    }

    cv::Mat crop = cropToBbox(image, det);
    if (crop.empty()) {
      output.detections.push_back(enriched);
      continue;
    }

    if (isTrafficLight(det)) {
      TrafficLightAttributes attrs = classifyTrafficLightState(crop, det);
      appendTrafficLightHypotheses(enriched, attrs);
    } else if (isCar(det)) {
      CarAttributes attrs = classifyCarBehavior(crop, det);
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

  return image(cv::Rect(x1_c, y1_c, x2_c - x1_c, y2_c - y1_c)).clone();
}

TrafficLightAttributes AttributeAssignerCore::classifyTrafficLightState(
  const cv::Mat & crop, const vision_msgs::msg::Detection2D & det) const
{
  TrafficLightAttributes attrs;
  const double min_sat = params_.traffic_light_min_saturation;
  const double min_val = params_.traffic_light_min_value;

  if (crop.rows < 3 || crop.cols < 2) {
    attrs.red = 0.25;
    attrs.green = 0.25;
    attrs.yellow = 0.25;
    attrs.left_turn = 0.25;
    return attrs;
  }

  cv::Mat hsv;
  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

  const double aspect = static_cast<double>(crop.rows) / static_cast<double>(crop.cols);

  // Vertical light (standard 3-stack): top = red, middle = yellow, bottom = green
  if (aspect > 1.2) {
    const int h = crop.rows;
    const int third = h / 3;
    cv::Scalar mean_top = cv::mean(hsv(cv::Rect(0, 0, crop.cols, third)));
    cv::Scalar mean_mid = cv::mean(hsv(cv::Rect(0, third, crop.cols, third)));
    cv::Scalar mean_bot = cv::mean(hsv(cv::Rect(0, 2 * third, crop.cols, h - 2 * third)));

    auto is_lit = [min_sat, min_val](const cv::Scalar & m) { return m[1] >= min_sat && m[2] >= min_val; };
    auto hue_red = [](double h) { return h <= 10.0 || h >= 170.0; };
    auto hue_yellow = [](double h) { return h > 10.0 && h <= 35.0; };
    auto hue_green = [](double h) { return h > 35.0 && h < 85.0; };

    const double h_top = mean_top[0], s_top = mean_top[1], v_top = mean_top[2];
    const double h_mid = mean_mid[0], s_mid = mean_mid[1], v_mid = mean_mid[2];
    const double h_bot = mean_bot[0], s_bot = mean_bot[1], v_bot = mean_bot[2];

    const bool lit_top = is_lit(mean_top);
    const bool lit_mid = is_lit(mean_mid);
    const bool lit_bot = is_lit(mean_bot);

    // One region lit -> assign that color
    if (lit_top && !lit_mid && !lit_bot) {
      if (hue_red(h_top)) {
        attrs.red = 0.92;
        attrs.green = 0.05;
        attrs.yellow = 0.05;
        attrs.left_turn = 0.05;
      } else if (hue_yellow(h_top)) {
        attrs.yellow = 0.92;
        attrs.red = 0.05;
        attrs.green = 0.05;
        attrs.left_turn = 0.05;
      } else if (hue_green(h_top)) {
        attrs.left_turn = 0.5;  // top green can be arrow
        attrs.green = 0.45;
        attrs.red = 0.05;
        attrs.yellow = 0.05;
      } else {
        attrs.red = 0.4;
        attrs.green = 0.3;
        attrs.yellow = 0.2;
        attrs.left_turn = 0.2;
      }
      return attrs;
    }
    if (!lit_top && lit_mid && !lit_bot) {
      attrs.yellow = 0.92;
      attrs.red = 0.05;
      attrs.green = 0.05;
      attrs.left_turn = 0.05;
      return attrs;
    }
    if (!lit_top && !lit_mid && lit_bot) {
      if (hue_green(h_bot)) {
        attrs.green = 0.92;
        attrs.red = 0.05;
        attrs.yellow = 0.05;
        attrs.left_turn = 0.05;
      } else if (hue_yellow(h_bot)) {
        attrs.yellow = 0.92;
        attrs.red = 0.05;
        attrs.green = 0.05;
        attrs.left_turn = 0.05;
      } else {
        attrs.green = 0.5;
        attrs.yellow = 0.3;
        attrs.red = 0.1;
        attrs.left_turn = 0.2;
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
        attrs.green = 0.05;
        attrs.left_turn = 0.05;
      } else if (best == 1) {
        attrs.yellow = 0.85;
        attrs.red = 0.08;
        attrs.green = 0.05;
        attrs.left_turn = 0.05;
      } else if (best == 2) {
        attrs.green = 0.85;
        attrs.red = 0.05;
        attrs.yellow = 0.08;
        attrs.left_turn = 0.05;
      }
      return attrs;
    }
  }

  // Horizontal or roughly square: single region or left-turn style
  if (aspect < 0.8 || (aspect >= 0.8 && aspect <= 1.2)) {
    cv::Scalar mean_all = cv::mean(hsv);
    const double h = mean_all[0], s = mean_all[1], v = mean_all[2];
    if (s >= min_sat && v >= min_val) {
      if (h <= 10.0 || h >= 170.0) {
        attrs.red = 0.88;
        attrs.green = 0.04;
        attrs.yellow = 0.04;
        attrs.left_turn = 0.04;
      } else if (h > 10.0 && h <= 35.0) {
        attrs.yellow = 0.88;
        attrs.red = 0.04;
        attrs.green = 0.04;
        attrs.left_turn = 0.04;
      } else if (h > 35.0 && h < 85.0) {
        attrs.green = 0.5;
        attrs.left_turn = 0.45;  // square green often arrow
        attrs.red = 0.03;
        attrs.yellow = 0.03;
      } else {
        attrs.red = 0.25;
        attrs.green = 0.25;
        attrs.yellow = 0.25;
        attrs.left_turn = 0.25;
      }
      return attrs;
    }
  }

  // No clear lit region
  attrs.red = 0.25;
  attrs.green = 0.25;
  attrs.yellow = 0.25;
  attrs.left_turn = 0.25;
  return attrs;
}

CarAttributes AttributeAssignerCore::classifyCarBehavior(
  const cv::Mat & crop, const vision_msgs::msg::Detection2D &) const
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
  const double min_red = params_.car_brake_min_red;
  const double min_bright = params_.car_brake_min_brightness;
  const double ah_lo = params_.car_amber_hue_lo;
  const double ah_hi = params_.car_amber_hue_hi;
  const double am_sat = params_.car_amber_min_saturation;
  const double am_val = params_.car_amber_min_value;

  // Bottom 25%: brake lights (red, bright)
  const int bottom_h = std::max(1, h / 4);
  cv::Mat bottom_region = crop(cv::Rect(0, h - bottom_h, w, bottom_h));
  cv::Scalar mean_bottom = cv::mean(bottom_region);
  const double b_r = mean_bottom[2];  // BGR order
  const double b_g = mean_bottom[1];
  const double b_b = mean_bottom[0];
  const double brightness = (b_r + b_g + b_b) / 3.0;
  if (b_r >= min_red && b_r >= b_g && b_r >= b_b && brightness >= min_bright) {
    attrs.braking = 0.85;
  } else {
    attrs.braking = 0.1;
  }

  // Left 25% and right 25%: amber for turn/hazard
  const int side_w = std::max(1, w / 4);
  cv::Mat left_region = crop(cv::Rect(0, 0, side_w, h));
  cv::Mat right_region = crop(cv::Rect(w - side_w, 0, side_w, h));

  cv::Mat left_hsv, right_hsv;
  cv::cvtColor(left_region, left_hsv, cv::COLOR_BGR2HSV);
  cv::cvtColor(right_region, right_hsv, cv::COLOR_BGR2HSV);

  cv::Scalar mean_left = cv::mean(left_hsv);
  cv::Scalar mean_right = cv::mean(right_hsv);

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
  det.results.push_back(makeHypothesis(std::string(kStatePrefix) + "left_turn_signal", attrs.left_turn));
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
