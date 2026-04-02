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

#include "attribute_assigner/traffic_light_classifier.hpp"

#include <algorithm>
#include <string>

#include <opencv2/imgproc.hpp>

namespace wato::perception::attribute_assigner
{

TrafficLightClassifier::TrafficLightClassifier(const Params & params)
: params_(params)
, ids_(params.class_ids.begin(), params.class_ids.end())
{}

bool TrafficLightClassifier::matches(const vision_msgs::msg::Detection2D & det) const
{
  for (const auto & result : det.results) {
    if (ids_.count(result.hypothesis.class_id) > 0) {
      return true;
    }
  }
  return false;
}

bool TrafficLightClassifier::matchesClassId(const std::string & class_id) const
{
  return ids_.count(class_id) > 0;
}

void TrafficLightClassifier::classify(const cv::Mat & crop, vision_msgs::msg::Detection2D & det) const
{
  // Safety-first prior — default to red when there's insufficient data to classify
  double red = 1.0, green = 0.0, yellow = 0.0;

  if (crop.rows < 3 || crop.cols < 2) {
    det.results.push_back(makeHypothesis(std::string(kPrefix) + "green", green));
    det.results.push_back(makeHypothesis(std::string(kPrefix) + "yellow", yellow));
    det.results.push_back(makeHypothesis(std::string(kPrefix) + "red", red));
    return;
  }

  // Convert to HSV once
  cv::Mat hsv;
  cv::cvtColor(crop, hsv, cv::COLOR_BGR2HSV);

  // Determine orientation: vertical (tall) or horizontal (wide) traffic light
  const bool is_horizontal = crop.cols > crop.rows;

  // Compute the center strip bounds along the shorter axis
  int row_start = 0, row_end = hsv.rows;
  int col_start = 0, col_end = hsv.cols;

  const double margin_frac = std::clamp(params_.strip_margin, 0.0, 0.45);

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
  const uint8_t min_val = static_cast<uint8_t>(params_.min_value);
  const uint8_t min_sat = static_cast<uint8_t>(params_.min_saturation);
  const uint8_t red_lo = static_cast<uint8_t>(params_.red_hue_lo);
  const uint8_t red_hi = static_cast<uint8_t>(params_.red_hue_hi);
  const uint8_t yellow_hi = static_cast<uint8_t>(params_.yellow_hue_hi);
  const uint8_t green_hi = static_cast<uint8_t>(params_.green_hue_hi);

  // Single pass over center strip pixels: count bright pixels per hue bucket
  int red_count = 0, yellow_count = 0, green_count = 0, bright_count = 0;

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
    }
  }

  const int total_pixels = (row_end - row_start) * (col_end - col_start);
  const int classified = red_count + yellow_count + green_count;

  // If too few bright pixels or none classified, default to red (safety-first for AV)
  if (bright_count >= total_pixels * 0.01 && bright_count >= 4 && classified > 0) {
    const double inv = 1.0 / static_cast<double>(classified);
    red = red_count * inv;
    green = green_count * inv;
    yellow = yellow_count * inv;
  }

  det.results.push_back(makeHypothesis(std::string(kPrefix) + "green", green));
  det.results.push_back(makeHypothesis(std::string(kPrefix) + "yellow", yellow));
  det.results.push_back(makeHypothesis(std::string(kPrefix) + "red", red));
}

BoxParams3D TrafficLightClassifier::get3DBoxParams(const vision_msgs::msg::Detection2D & /* det */) const
{
  BoxParams3D box;
  box.width = params_.box_width;
  box.height = params_.box_height;
  box.length = params_.box_length;
  box.assumed_depth = params_.assumed_depth;
  box.use_width_for_depth = false;  // Use height for depth estimation
  return box;
}

MarkerColor TrafficLightClassifier::getMarkerColor(const vision_msgs::msg::Detection2D & det) const
{
  MarkerColor color;
  color.a = 0.8f;

  double best_score = -1.0;
  std::string best_state;
  for (const auto & result : det.results) {
    const std::string & cid = result.hypothesis.class_id;
    if ((cid == "state:red" || cid == "state:yellow" || cid == "state:green") && result.hypothesis.score > best_score) {
      best_score = result.hypothesis.score;
      best_state = cid;
    }
  }

  if (best_state == "state:red") {
    color.r = 1.0f;
    color.g = 0.0f;
    color.b = 0.0f;
  } else if (best_state == "state:yellow") {
    color.r = 1.0f;
    color.g = 1.0f;
    color.b = 0.0f;
  } else if (best_state == "state:green") {
    color.r = 0.0f;
    color.g = 1.0f;
    color.b = 0.0f;
  }

  return color;
}

}  // namespace wato::perception::attribute_assigner
