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

#include <algorithm>
#include <chrono>
#include <cmath>
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

bool BEVFusionCore::initialize()
{
  // Must load this plugin before create_core() — TRT will fail to deserialize head.bbox.plan without it
  if (dlopen("libcustom_layernorm.so", RTLD_NOW) == nullptr) {
    return false;
  }

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

  ::bevfusion::lidar::SCNParameter scn;
  scn.voxelization = voxel;
  scn.model = config_.lidar_backbone_onnx;
  scn.order = scn_order;
  scn.precision = precision;

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
    return false;
  }

  // dedicated stream: BEVFusion's GPU ops stay ordered in their own queue, not on the null (default) stream shared by everything else
  if (cudaStreamCreate(&stream_) != cudaSuccess) {
    pipeline_.reset();
    return false;
  }

  initialized_ = true;
  return true;
}

std::vector<BoundingBox> BEVFusionCore::infer(
  const std::vector<const unsigned char *> & camera_images, const std::vector<float> & lidar_points, int num_points)
{
  /**
 * TODO(bevfusion_team)
 * Overview: Convert data to required format and feed it to GPU pipeline (needs LiDAR data to be FP16)
 *
 * Key steps:
 *   1. Convert the input `lidar_points` (vector of `float`) into a vector of `nvtype::half`.
 *   2. Call `pipeline_->forward(images.data(), lidar_half.data(), num_points, stream_)`.
 *   3. Return the `std::vector<BoundingBox>` result.
 */
}

void BEVFusionCore::updateCalibration(
  const std::vector<float> & camera_to_lidar,
  const std::vector<float> & camera_intrinsics,
  const std::vector<float> & lidar_to_camera,
  const std::vector<float> & img_aug_matrix)
{
  /**
 * TODO(bevfusion_team)
 * Overview: Feed the current physical positions of the cameras (extrinsics) and lens properties (instrinsics) to
 *           the BEVFusion Core (pipeline_). This is done by calling pipeline_->update() with the appropriate
 *           parameters. Note: Extrinsic and instrinsics are static, so only run this once at startup or when calibration
 *           changes. MUST be called before the first forward() call.
 *
 * Call core_->update(camera2lidar.data(), camera_intrinsics.data(), lidar2image.data(), img_aug_matrix.data(), stream_)
 * and that I think that should be it.
 */
}

}  // namespace wato::perception::bevfusion
