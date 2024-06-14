#ifndef CAMERA_UTILS_HPP_
#define CAMERA_UTILS_HPP_

#include <opencv2/opencv.hpp>

namespace CameraUtils {
// Swaps the channels of an image from HWC to CHW format
void hwc_img_2_chw_data(const cv::Mat& hwc_img, float* data);
cv::Mat resize_image_aspect_ratio(const cv::Mat& original_image, int max_width, int max_height);
cv::Mat resize_with_padding(const cv::Mat& original_image, int target_width, int target_height);
cv::Mat resize_from_center(const cv::Mat& original_image, int target_width, int target_height);
};  // namespace CameraUtils

#endif  // CAMERA_UTILS_HPP_