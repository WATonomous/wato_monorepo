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

#include "bevfusion/bevfusion_core.hpp"

#include <dlfcn.h>
#include <sys/wait.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/core.hpp>

namespace wato::perception::bevfusion
{

BEVFusionCore::BEVFusionCore(const BEVFusionInputConfig & config)
: config_(config)
{}

BEVFusionCore::~BEVFusionCore()
{
  if (stream_ != nullptr) {
    cudaStreamDestroy(stream_);
  }
}

bool BEVFusionCore::isInitialized() const
{
  return initialized_;
}

bool BEVFusionCore::hasCalibration() const
{
  return has_calibration_;
}

bool BEVFusionCore::checkModelFilesExist() const
{
  bool result = true;
  if (!std::filesystem::exists(config_.camera_backbone_plan)) {
    std::cerr << "[BEVFusionCore] MISSING: camera_backbone_plan='" << config_.camera_backbone_plan << "'" << std::endl;
    result = false;
  }
  if (!std::filesystem::exists(config_.camera_vtransform_plan)) {
    std::cerr << "[BEVFusionCore] MISSING: camera_vtransform_plan='" << config_.camera_vtransform_plan << "'"
              << std::endl;
    result = false;
  }
  if (!std::filesystem::exists(config_.fuser_plan)) {
    std::cerr << "[BEVFusionCore] MISSING: fuser_plan='" << config_.fuser_plan << "'" << std::endl;
    result = false;
  }
  if (!std::filesystem::exists(config_.head_bbox_plan)) {
    std::cerr << "[BEVFusionCore] MISSING: head_bbox_plan='" << config_.head_bbox_plan << "'" << std::endl;
    result = false;
  }
  if (!std::filesystem::exists(config_.lidar_backbone_onnx)) {
    std::cerr << "[BEVFusionCore] MISSING: lidar_backbone_onnx='" << config_.lidar_backbone_onnx << "'" << std::endl;
    result = false;
  }
  return result;
}

bool BEVFusionCore::buildTRTEngines() const
{
  return false;
}

bool BEVFusionCore::initialize()
{
  // Must load this plugin before create_core() — TRT will fail to deserialize head.bbox.plan without it
  if (dlopen("libcustom_layernorm.so", RTLD_NOW) == nullptr) {
    std::cerr << "[BEVFusionCore] Failed to load libcustom_layernorm.so: " << dlerror() << std::endl;
    return false;
  }

  // Check if all required model files exist, and build missing TensorRT engines if needed
  if (!checkModelFilesExist()) {
    std::cout << "[BEVFusionCore] Building TRT engines. Please hold for several seconds..." << std::endl;
    if (!buildTRTEngines()) {
      std::cerr << "[BEVFusionCore] Failed to build TensorRT engines" << std::endl;
      return false;
    }
  }

  // Ensure that all required model files exist after the build step
  if (!checkModelFilesExist()) {
    std::cerr << "[BEVFusionCore] Missing plan/onnx files, cannot create core!" << std::endl;
    return false;
  }

  // Camera normalization: resize + mean/std normalization applied to each input image before the backbone
  ::bevfusion::camera::NormalizationParameter norm;
  norm.image_width = config_.image_width;
  norm.image_height = config_.image_height;
  norm.num_camera = config_.num_cameras;
  norm.output_width = config_.norm_output_width;
  norm.output_height = config_.norm_output_height;
  norm.resize_lim = config_.resize_lim;
  norm.interpolation = (config_.interpolation == "nearest") ? ::bevfusion::camera::Interpolation::Nearest
                                                            : ::bevfusion::camera::Interpolation::Bilinear;
  norm.method = ::bevfusion::camera::NormMethod::mean_std(
    config_.norm_mean.data(), config_.norm_std.data(), config_.norm_scale, config_.norm_bias);

  // Camera geometry: BEV grid bounds and feature map dimensions for lifting image features into 3D space
  ::bevfusion::camera::GeometryParameter geom;
  geom.xbound = {config_.xbound[0], config_.xbound[1], config_.xbound[2]};
  geom.ybound = {config_.ybound[0], config_.ybound[1], config_.ybound[2]};
  geom.zbound = {config_.zbound[0], config_.zbound[1], config_.zbound[2]};
  geom.dbound = {config_.dbound[0], config_.dbound[1], config_.dbound[2]};
  geom.geometry_dim = {config_.geometry_dim[0], config_.geometry_dim[1], config_.geometry_dim[2]};
  geom.feat_width = config_.feat_width;
  geom.feat_height = config_.feat_height;
  geom.image_width = config_.image_width;
  geom.image_height = config_.image_height;
  geom.num_camera = config_.num_cameras;

  // LiDAR voxelization: divides the point cloud into a 3D grid; grid_size is derived from range and voxel size
  nvtype::Float3 min_range = {config_.min_range[0], config_.min_range[1], config_.min_range[2]};
  nvtype::Float3 max_range = {config_.max_range[0], config_.max_range[1], config_.max_range[2]};
  nvtype::Float3 voxel_size = {config_.voxel_size[0], config_.voxel_size[1], config_.voxel_size[2]};

  ::bevfusion::lidar::VoxelizationParameter voxel;
  voxel.min_range = min_range;
  voxel.max_range = max_range;
  voxel.voxel_size = voxel_size;
  voxel.grid_size = ::bevfusion::lidar::VoxelizationParameter::compute_grid_size(max_range, min_range, voxel_size);
  voxel.num_feature = config_.num_features;
  voxel.max_voxels = config_.max_voxels;
  voxel.max_points_per_voxel = config_.max_points_per_voxel;
  voxel.max_points = config_.max_points;

  const auto scn_order =
    (config_.scn_order == "ZYX") ? ::bevfusion::lidar::CoordinateOrder::ZYX : ::bevfusion::lidar::CoordinateOrder::XYZ;
  const auto precision =
    (config_.precision == "int8") ? ::bevfusion::lidar::Precision::Int8 : ::bevfusion::lidar::Precision::Float16;

  // Sparse conv backbone (SCN): processes only non-empty voxels for efficient LiDAR feature extraction
  ::bevfusion::lidar::SCNParameter scn;
  scn.voxelization = voxel;
  scn.model = config_.lidar_backbone_onnx;
  scn.order = scn_order;
  scn.precision = precision;

  // Detection head: decodes BEV feature map into 3D bounding boxes and filters by confidence + center range
  ::bevfusion::head::transbbox::TransBBoxParameter transbbox;
  transbbox.model = config_.head_bbox_plan;
  transbbox.out_size_factor = config_.out_size_factor;
  transbbox.voxel_size = {config_.transbbox_voxel_size[0], config_.transbbox_voxel_size[1]};
  transbbox.pc_range = {config_.transbbox_pc_range[0], config_.transbbox_pc_range[1]};
  transbbox.post_center_range_start = {
    config_.post_center_range_start[0], config_.post_center_range_start[1], config_.post_center_range_start[2]};
  transbbox.post_center_range_end = {
    config_.post_center_range_end[0], config_.post_center_range_end[1], config_.post_center_range_end[2]};
  transbbox.confidence_threshold = config_.confidence_threshold;
  transbbox.sorted_bboxes = config_.sorted_bboxes;

  // Deserialize all TensorRT engines and allocate GPU buffers for the full pipeline
  ::bevfusion::CoreParameter param;
  param.camera_model = config_.camera_backbone_plan;
  param.camera_vtransform = config_.camera_vtransform_plan;
  param.geometry = geom;
  param.normalize = norm;
  param.lidar_scn = scn;
  param.transfusion = config_.fuser_plan;
  param.transbbox = transbbox;

  pipeline_ = ::bevfusion::create_core(param);
  if (pipeline_ == nullptr) {
    std::cerr
      << "[BEVFusionCore] Failed to create BEVFusion pipeline. Ensure model files exist and GPU memory is sufficient."
      << std::endl;
    return false;
  }

  // dedicated stream: BEVFusion's GPU ops stay ordered in their own queue, not on the null (default) stream shared by everything else
  if (cudaStreamCreate(&stream_) != cudaSuccess) {
    std::cerr << "[BEVFusionCore] cudaStreamCreate failed." << std::endl;
    pipeline_.reset();
    return false;
  }

  initialized_ = true;
  return true;
}

std::vector<BoundingBox> BEVFusionCore::infer(
  const std::vector<const unsigned char *> & camera_images, const std::vector<float> & lidar_points, int num_points)
{
  if (!initialized_) {
    std::cerr << "Error: BEVFusionCore::infer called before initialization." << std::endl;
    return {};  // guard check, return empty vector if pipeline is not initialized
  }
  if (!has_calibration_) {
    std::cerr << "Error: BEVFusionCore::infer called before calibration was updated." << std::endl;
    return {};
  }

  // Basic input validation / diagnostics to catch malformed inputs before hitting vendor code
  std::cout << "[BEVFusionCore] infer called: camera_images=" << camera_images.size()
            << ", expected_num_cameras=" << config_.num_cameras << ", lidar_points.size=" << lidar_points.size()
            << ", num_points=" << num_points << std::endl;

  for (size_t i = 0; i < camera_images.size(); ++i) {
    if (camera_images[i] == nullptr) {
      std::cerr << "[BEVFusionCore] ERROR: camera_images[" << i << "] is null" << std::endl;
      return {};
    }
  }

  const size_t expected_lidar_floats = static_cast<size_t>(num_points) * static_cast<size_t>(config_.num_features);
  if (lidar_points.size() != expected_lidar_floats) {
    std::cerr << "[BEVFusionCore] WARNING: lidar_points.size() (" << lidar_points.size()
              << ") != num_points * num_features (" << expected_lidar_floats << ")" << std::endl;
    // If lidar_points contains more data than expected, we trim locally to avoid downstream issues.
    if (lidar_points.size() < expected_lidar_floats) {
      std::cerr << "[BEVFusionCore] ERROR: insufficient LiDAR data for declared num_points; aborting infer."
                << std::endl;
      return {};
    }
  }

  // Convert LiDAR points to FP16 from any format (FP32, FP16, or INT8)
  // Notes:
  // - lidar_points.size() is the total # of floats in the flat array of 5 features per lidar point
  // - We set lidar_half as a vector of __half and not nvtype::half because __float2half returns __half format.
  std::vector<__half> lidar_half(lidar_points.size());
  for (size_t i = 0; i < lidar_points.size(); ++i) {
    lidar_half[i] = __float2half(lidar_points[i]);
  }

  // Bevfusion forward pass call
  // Notes:
  // - camera_images.data(): gives the array of image pointers. Each pointer points to the start of the data for one camera.
  //   The `const_cast` is used because the vendor library expects a non-const pointer, even though it doesn't modify the image data.
  // - lidar_half.data(): gives a pointer to the first element of the vector containing points in __half format.
  //   The `reinterpret_cast` is used to cast this pointer to `const nvtype::half*`, which is the expected type for the vendor library API.
  //   Could have also used memcpy to manually copy bits from __half to nvtype::half since they are bitwise identical.
  // - Use .data() for underlying array
  auto detections = pipeline_->forward(
    const_cast<const unsigned char **>(camera_images.data()),
    reinterpret_cast<const nvtype::half *>(lidar_half.data()),
    num_points,
    stream_);

  std::cout << "[BEVFusionCore] pipeline_->forward returned " << detections.size() << " detections" << std::endl;
  if (detections.empty()) {
    std::cerr << "[BEVFusionCore] WARNING: pipeline returned zero detections. has_calibration=" << has_calibration_
              << ", pipeline_ptr=" << pipeline_.get() << ", precision=" << config_.precision << std::endl;
  }

  // map vendor type to our bounding box type
  std::vector<BoundingBox> result;
  result.reserve(detections.size());
  for (const auto & d : detections) {
    BoundingBox box;
    box.position = {d.position.x, d.position.y, d.position.z};
    box.size = {d.size.w, d.size.l, d.size.h};
    box.velocity = {d.velocity.vx, d.velocity.vy};
    box.z_rotation = d.z_rotation;
    box.score = d.score;
    box.id = d.id;
    result.push_back(box);
  }
  return result;
}

void BEVFusionCore::updateCalibration(
  const std::vector<float> & camera_to_lidar,
  const std::vector<float> & camera_intrinsics,
  const std::vector<float> & lidar_to_image_projection,
  const std::vector<float> & img_aug_matrix)
{
  if (!initialized_) {
    std::cerr << "Error: BEVFusionCore::updateCalibration called before initialization." << std::endl;
    return;
  }
  pipeline_->update(
    camera_to_lidar.data(), camera_intrinsics.data(), lidar_to_image_projection.data(), img_aug_matrix.data(), stream_);
  has_calibration_ = true;
}

}  // namespace wato::perception::bevfusion
