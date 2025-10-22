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

#ifndef CAMERA_UTILS_HPP_
#define CAMERA_UTILS_HPP_

#include <opencv2/opencv.hpp>

namespace CameraUtils
{
// Swaps the channels of an image from HWC to CHW format
void hwc_img_2_chw_data(const cv::Mat & hwc_img, float * data);
cv::Mat resize_image_aspect_ratio(const cv::Mat & original_image, int max_width, int max_height);
cv::Mat resize_with_padding(const cv::Mat & original_image, int target_width, int target_height);
cv::Mat resize_from_center(const cv::Mat & original_image, int target_width, int target_height);
};  // namespace CameraUtils

#endif  // CAMERA_UTILS_HPP_
