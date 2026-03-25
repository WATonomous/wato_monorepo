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
#include <utility>

#include <opencv2/core.hpp>

namespace wato::perception::attribute_assigner
{

AttributeAssignerCore::AttributeAssignerCore(double min_detection_confidence)
: min_detection_confidence_(min_detection_confidence)
{}

void AttributeAssignerCore::addClassifier(std::unique_ptr<AttributeClassifier> classifier)
{
  classifiers_.push_back(std::move(classifier));
}

vision_msgs::msg::Detection2DArray AttributeAssignerCore::process(
  const cv::Mat & image, const vision_msgs::msg::Detection2DArray & input)
{
  const auto start = std::chrono::steady_clock::now();

  vision_msgs::msg::Detection2DArray output;
  output.header = input.header;
  output.detections.reserve(input.detections.size());

  if (image.empty()) {
    output.detections = input.detections;
    last_processing_time_ms_ = 0.0;
    processed_count_ += input.detections.size();
    return output;
  }

  for (const auto & det : input.detections) {
    vision_msgs::msg::Detection2D enriched = det;

    const AttributeClassifier * classifier = findClassifier(det);

    if (classifier == nullptr || getBestScore(det) < min_detection_confidence_) {
      output.detections.push_back(enriched);
      continue;
    }

    cv::Mat crop = cropToBbox(image, det);
    if (crop.empty()) {
      output.detections.push_back(enriched);
      continue;
    }

    classifier->classify(crop, enriched);
    output.detections.push_back(enriched);
  }

  const auto end = std::chrono::steady_clock::now();
  last_processing_time_ms_ = std::chrono::duration<double, std::milli>(end - start).count();
  processed_count_ += input.detections.size();

  return output;
}

const AttributeClassifier * AttributeAssignerCore::findClassifier(const vision_msgs::msg::Detection2D & det) const
{
  for (const auto & classifier : classifiers_) {
    if (classifier->matches(det)) {
      return classifier.get();
    }
  }
  return nullptr;
}

const AttributeClassifier * AttributeAssignerCore::findClassifierByClassId(const std::string & class_id) const
{
  for (const auto & classifier : classifiers_) {
    if (classifier->matchesClassId(class_id)) {
      return classifier.get();
    }
  }
  return nullptr;
}

uint64_t AttributeAssignerCore::getProcessedCount() const
{
  return processed_count_;
}

double AttributeAssignerCore::getLastProcessingTimeMs() const
{
  return last_processing_time_ms_;
}

std::string AttributeAssignerCore::getBestClassId(const vision_msgs::msg::Detection2D & det)
{
  if (det.results.empty()) {
    return "";
  }

  const auto best = std::max_element(det.results.begin(), det.results.end(), [](const auto & a, const auto & b) {
    return a.hypothesis.score < b.hypothesis.score;
  });

  return best->hypothesis.class_id;
}

double AttributeAssignerCore::getBestScore(const vision_msgs::msg::Detection2D & det)
{
  if (det.results.empty()) {
    return 0.0;
  }

  const auto best = std::max_element(det.results.begin(), det.results.end(), [](const auto & a, const auto & b) {
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

  return image(cv::Rect(x1_c, y1_c, x2_c - x1_c, y2_c - y1_c));
}

Projection3D AttributeAssignerCore::projectTo3D(
  const CameraIntrinsics & cam, const vision_msgs::msg::Detection2D & det, const BoxParams3D & box)
{
  const double bbox_w_pixels = det.bbox.size_x;
  const double bbox_h_pixels = det.bbox.size_y;
  const double u = det.bbox.center.position.x;
  const double v = det.bbox.center.position.y;

  // Estimate depth from known physical size and pixel footprint
  double estimated_depth;
  if (box.use_width_for_depth) {
    estimated_depth = (box.width * cam.fy) / bbox_w_pixels;
  } else {
    estimated_depth = (box.height * cam.fy) / bbox_h_pixels;
  }

  const double depth = (estimated_depth > 3.0 && estimated_depth < 150.0) ? estimated_depth : box.assumed_depth;

  // Clamp 3D dimensions so their projection doesn't exceed the 2D bbox
  const double max_real_width = (bbox_w_pixels * depth) / cam.fx;
  const double max_real_height = (bbox_h_pixels * depth) / cam.fy;

  Projection3D result;
  result.size_x = std::min(box.width, max_real_width);
  result.size_y = std::min(box.length, max_real_width);  // length projects along width axis
  result.size_z = std::min(box.height, max_real_height);

  // Unproject to 3D ray in camera frame, scale to estimated depth
  const double x_cam = (u - cam.cx) / cam.fx;
  const double y_cam = (v - cam.cy) / cam.fy;
  const double z_cam = 1.0;
  const double ray_length = std::sqrt(x_cam * x_cam + y_cam * y_cam + z_cam * z_cam);

  result.x = (x_cam / ray_length) * depth;
  result.y = (y_cam / ray_length) * depth;
  result.z = (z_cam / ray_length) * depth;

  return result;
}

}  // namespace wato::perception::attribute_assigner
