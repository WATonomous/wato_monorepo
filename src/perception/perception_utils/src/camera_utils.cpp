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

#include "perception_utils/camera_utils.hpp"

#include <opencv2/opencv.hpp>

// Swaps the channels of an image from HWC to CHW format
void CameraUtils::hwc_img_2_chw_data(const cv::Mat & hwc_img, float * data)
{
  int rows = hwc_img.rows;
  int cols = hwc_img.cols;
  int chs = hwc_img.channels();
  for (int i = 0; i < chs; ++i) {
    cv::extractChannel(hwc_img, cv::Mat(rows, cols, CV_32FC1, data + i * rows * cols), i);
  }
}

cv::Mat CameraUtils::resize_image_aspect_ratio(const cv::Mat & original_image, int max_width, int max_height)
{
  int original_width = original_image.cols;
  int original_height = original_image.rows;
  double original_aspect_ratio = (double)original_width / original_height;

  int new_width, new_height;
  double max_aspect_ratio = (double)max_width / max_height;

  if (original_aspect_ratio > max_aspect_ratio) {
    // Width is the limiting factor
    new_width = max_width;
    new_height = static_cast<int>(max_width / original_aspect_ratio);
  } else {
    // Height is the limiting factor
    new_height = max_height;
    new_width = static_cast<int>(max_height * original_aspect_ratio);
  }

  cv::Mat resized_image;
  cv::resize(original_image, resized_image, cv::Size(new_width, new_height));

  return resized_image;
}

cv::Mat CameraUtils::resize_with_padding(const cv::Mat & original_image, int target_width, int target_height)
{
  int original_width = original_image.cols;
  int original_height = original_image.rows;

  double target_ratio = (double)target_width / target_height;
  double original_ratio = (double)original_width / original_height;

  int new_width, new_height;

  if (original_ratio > target_ratio) {
    // Original is wider. Fit to width and pad height.
    new_width = target_width;
    new_height = static_cast<int>(original_height * (static_cast<double>(target_width) / original_width));
  } else {
    // Original is taller. Fit to height and pad width.
    new_height = target_height;
    new_width = static_cast<int>(original_width * (static_cast<double>(target_height) / original_height));
  }

  cv::Mat resized_image;
  cv::resize(original_image, resized_image, cv::Size(new_width, new_height));

  int top = (target_height - new_height) / 2;
  int bottom = target_height - new_height - top;
  int left = (target_width - new_width) / 2;
  int right = target_width - new_width - left;

  cv::Mat padded_image;
  cv::copyMakeBorder(resized_image, padded_image, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

  return padded_image;
}

cv::Mat CameraUtils::resize_from_center(const cv::Mat & original_image, int target_width, int target_height)
{
  int original_width = original_image.cols;
  int original_height = original_image.rows;

  // Calculate the new height maintaining the aspect ratio
  double target_ratio = (double)target_width / target_height;
  int new_height = static_cast<int>(original_width / target_ratio);

  // Calculate the cropping area
  int startY = (original_height - new_height) / 2;

  // Crop the image from the center
  cv::Rect roi(0, startY, original_width, new_height);
  cv::Mat cropped_image = original_image(roi);

  // Resize the cropped image
  cv::Mat resized_image;
  cv::resize(cropped_image, resized_image, cv::Size(target_width, target_height));

  return resized_image;
}
