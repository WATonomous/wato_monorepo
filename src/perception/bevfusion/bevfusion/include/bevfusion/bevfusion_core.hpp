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

#pragma once

#include <cuda_fp16.h>
#include <cuda_runtime.h>

#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include <bevfusion/bevfusion.hpp>
#include <opencv2/core/mat.hpp>

namespace wato::perception::bevfusion
{

/**
 * @brief Input parameters for CUDA-BEVFusion
 */
struct BEVFusionInputConfig
{
  //  --- Model Files (paths to TensorRT .plan files and ONNX model) ---
  std::string model_dir;
  std::string build_dir;
  std::string camera_backbone_plan;  // Extracts image features
  std::string camera_vtransform_plan;  // Projects image features to BEV
  std::string fuser_plan;  // Fuses image and LiDAR BEV features
  std::string head_bbox_plan;  // Predicts 3D bounding boxes
  std::string lidar_backbone_onnx;  // Extracts LiDAR features

  // --- Camera Normalization & Preprocessing ---
  int image_width = 1600;  // Input image resolution
  int image_height = 900;
  int norm_output_width = 704;  // Output resolution after resize (Network input)
  int norm_output_height = 256;
  int num_cameras = 6;
  float resize_lim =
    0.48f;  // Min scaling ratio limit - the shortest side is resized to at least 'resize_lim' of original side
  std::string interpolation = "bilinear";  // Interpolation method for resizing
  std::vector<float> norm_mean = {0.485f, 0.456f, 0.406f};  // Mean subtraction value per RGB channel
  std::vector<float> norm_std = {0.229f, 0.224f, 0.225f};  // Std deviation value per RGB channel
  float norm_scale = 1.0f / 255.0f;  // Input image pixel scaling factor ([0,255] -> [0,1])
  float norm_bias = 0.0f;  // Bias offset applied to normalized pixels

  // --- Lidar Voxelization & SCN ---
  std::vector<float> min_range = {-54.0f, -54.0f, -5.0f};  // Min 3D range (x,y,z) in meters for voxelization
  std::vector<float> max_range = {54.0f, 54.0f, 3.0f};  // Max 3D range
  std::vector<float> voxel_size = {0.075f, 0.075f, 0.2f};  // Size of each voxel (x,y,z) in meters
  int max_points_per_voxel = 10;  // Max # of points to sample in 1 voxel
  int max_points = 300000;  // Upper limit on total # of lidar points processed
  int max_voxels = 160000;  // Upper limit on total # of voxels generated on the grid
  int num_features = 5;  // Dimensions per lidar point (x, y, z, intensity, ring)
  std::string scn_order = "XYZ";  // SCN coordinate order constraint
  std::string precision = "int8";  // "fp16" or "int8". Used by LiDAR ONNX parser.

  // --- Camera Geometry Mapping (Depth & Grid) ---
  std::vector<float> xbound = {-54.0f, 54.0f, 0.3f};  // BEV grid X limits [min, max, step] in meters
  std::vector<float> ybound = {-54.0f, 54.0f, 0.3f};  // BEV grid Y limits
  std::vector<float> zbound = {-10.0f, 10.0f, 20.0f};  // BEV grid Z limits
  std::vector<float> dbound = {1.0f, 60.0f, 0.5f};  // Depth bin limits for feature projection
  std::vector<int> geometry_dim = {360, 360, 80};  // Voxel dimensions (width, height, depth_bin) in BEV space
  int feat_width = 88;  // Feature map width after projection (by camera backbone)
  int feat_height = 32;  // Feature map height after projection (by camera backbone)

  // --- Bounding Box Post-processing & Head ---
  int out_size_factor = 8;  // Resolution downsampling factor of the detection head output relative to voxels
  std::vector<float> transbbox_pc_range = {
    -54.0f, -54.0f};  // 2D point cloud start boundary [min_x, min_y] for the head
  std::vector<float> transbbox_voxel_size = {0.075f, 0.075f};  // 2D voxel size [dX, dY] in the horizontal plane
  std::vector<float> post_center_range_start = {-61.2f, -61.2f, -10.0f};  // BBox center filter lower boundary [X, Y, Z]
  std::vector<float> post_center_range_end = {61.2f, 61.2f, 10.0f};  // BBox center filter upper boundary [X, Y, Z]
  float confidence_threshold = 0.3f;  // Detection score threshold (detections below this score are discarded)
  bool sorted_bboxes = true;  // True to sort bounding boxes descending by confidence score
};

/**
 * @brief 3D position of the bounding box
 */
struct Position
{
  float x, y, z;
};

/**
 * @brief Dimensions of the bounding box
 */
struct Size
{
  float w, l, h;  // x, y, z
};

/**
 * @brief Velocity of the bounding box
 */
struct Velocity
{
  float vx, vy;
};

/**
 * @brief 3D bounding box
 */
struct BoundingBox
{
  Position position;
  Size size;
  Velocity velocity;
  float z_rotation;
  float score;
  int id;
};

/**
 * @brief Core logic for BEVFusion.
 *
 * Note: Use ::bevfusion when referencing anything from the original CUDA-BEVFusion repository.
 * Use wato::perception::bevfusion (handled by namespaces) when referencing anything from wato's bevfusion node.
 *
 * Note: nvtype is defined in the original CUDA-BEVFusion repository under /CUDA-BEVFusion/src/common/dtype.hpp and
 * can be used as nvtype:: or ::nvtype::.
 */
class BEVFusionCore
{
public:
  /**
   * @brief Construct the core with a configuration
   * @param config Configuration for BEVFusion
   */
  explicit BEVFusionCore(const BEVFusionInputConfig & config);
  ~BEVFusionCore();

  /**
   * @brief Create the CUDA-BEVFusion pipeline and deserialize TensorRT engines.
   * @return true if initialization was successful, false otherwise
   */
  bool initialize();

  /**
   * @brief Main inference entry point. Processes one synchronized frame of camera and LiDAR data.
   *        Converts data to required format and feed it to GPU pipeline (needs LiDAR data to be FP16).
   * @param camera_images Vector of camera images
   * @param lidar_points Vector of LiDAR points
   * @param num_points Number of LiDAR points
   * @return Vector of detected 3D bounding boxes
   *
   * @note lidar_points is a flattened 1D vector containing all the raw LiDAR data for a single frame
   *       - lidar_points.size() equals num_points * config_.num_features.
   *       - The vector looks like [x1, y1, z1, i1, r1, x2, y2, z2, i2, r2, ...],
   *         where (x,y,z) are coordinates, i is intensity and r is ring number.
   * @note Preconditions:
   *       - initialize() called and returned true; pipeline_ and stream_ are live.
   *       - updateCalibration() called at least once; pipeline needs camera extrinsics/intrinsics before its first forward pass
   *         to relate cameras to the LiDAR.
   */
  std::vector<BoundingBox> infer(
    const std::vector<const unsigned char *> & camera_images, const std::vector<float> & lidar_points, int num_points);

  /**
   * @brief Update the tansformation matrices mapping cameras to LiDARs
   * @param camera_to_lidar 4x4 camera to LiDAR transformation matrix
   * @param camera_instrinsics 3x3 camera intrinsics matrix
   * @param lidar_to_image_projection 4x4 LiDAR to image projection matrix
   * @param img_aug_matrix 4x4 image augmentation matrix
   *
   * @note Preconditions:
   *       - initialize() called and returned true; pipeline_ and stream_ are live.
   */
  void updateCalibration(
    const std::vector<float> & camera_to_lidar,
    const std::vector<float> & camera_intrinsics,
    const std::vector<float> & lidar_to_image_projection,
    const std::vector<float> & img_aug_matrix);

  /**
   * @brief Check if the core is initialized
   * @return true if initialized, false otherwise
   */
  bool isInitialized() const;

  /**
   * @brief Check if the calibration is initialized
   * @return true if calibration is initialized, false otherwise
   */
  bool hasCalibration() const;

private:
  /**
   * @brief Check if all model files exist
   * @return true if all model files exist, false otherwise
   */
  bool checkModelFilesExist() const;

  /**
   * @brief Build TRT engines
   * @return true if all engines were built successfully, false otherwise
   */
  bool buildTRTEngines() const;

  /**
   * @brief Compile TRT engine from ONNX file
   * @param name Name of the model
   * @param onnx_file Path to ONNX file
   * @param plan_file Path to plan file
   * @return true if successful, false otherwise
   */
  bool compileTrtModel(
    const std::string & name, const std::filesystem::path & onnx_file, const std::filesystem::path & plan_file) const;

  BEVFusionInputConfig config_;
  bool initialized_ = false;
  bool has_calibration_ = false;
  cudaStream_t stream_ = nullptr;
  std::shared_ptr<::bevfusion::Core>
    pipeline_;  // Reference to CUDA-BEVFusion pipeline, not wato's BEVFusionCore class.
};

}  // namespace wato::perception::bevfusion
