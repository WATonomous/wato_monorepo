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

#include <cuda_runtime_api.h>
#include <NvInfer.h>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <fstream>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "prediction_ml/mtr_backend.hpp"

// This translation unit is compiled only when PREDICTION_ML_ENABLE_TENSORRT is set.
namespace prediction_ml
{
namespace detail
{
namespace
{

constexpr int64_t kTargetCapacity = 8;
constexpr int64_t kContextAgentCapacity = 128;
constexpr int64_t kHistorySteps = 11;
constexpr int64_t kObjectFeatureCount = 29;
constexpr int64_t kMapPolylineCapacity = 768;
constexpr int64_t kPointsPerPolyline = 20;
constexpr int64_t kMapFeatureCount = 9;

class TensorRtLogger final : public nvinfer1::ILogger
{
public:
  void log(const Severity severity, const char * message) noexcept override
  {
    if (severity <= Severity::kERROR && message != nullptr) {
      last_message_ = message;
    }
  }

  const std::string & lastMessage() const
  {
    return last_message_;
  }

private:
  std::string last_message_;
};

class DeviceBuffer
{
public:
  DeviceBuffer() = default;
  DeviceBuffer(const DeviceBuffer &) = delete;
  DeviceBuffer & operator=(const DeviceBuffer &) = delete;

  DeviceBuffer(DeviceBuffer && other) noexcept
  : data_(std::exchange(other.data_, nullptr))
  , bytes_(std::exchange(other.bytes_, 0))
  {}

  DeviceBuffer & operator=(DeviceBuffer && other) noexcept
  {
    if (this != &other) {
      reset();
      data_ = std::exchange(other.data_, nullptr);
      bytes_ = std::exchange(other.bytes_, 0);
    }
    return *this;
  }

  ~DeviceBuffer()
  {
    reset();
  }

  cudaError_t allocate(const std::size_t bytes)
  {
    reset();
    bytes_ = bytes;
    if (bytes_ == 0) {
      return cudaSuccess;
    }
    return cudaMalloc(&data_, bytes_);
  }

  void * data() const
  {
    return data_;
  }

  std::size_t bytes() const
  {
    return bytes_;
  }

private:
  void reset()
  {
    if (data_ != nullptr) {
      (void)cudaFree(data_);
      data_ = nullptr;
    }
    bytes_ = 0;
  }

  void * data_{nullptr};
  std::size_t bytes_{0};
};

class CudaStream
{
public:
  CudaStream() = default;
  CudaStream(const CudaStream &) = delete;
  CudaStream & operator=(const CudaStream &) = delete;

  ~CudaStream()
  {
    if (stream_ != nullptr) {
      (void)cudaStreamDestroy(stream_);
    }
  }

  cudaError_t create()
  {
    return cudaStreamCreate(&stream_);
  }

  cudaStream_t get() const
  {
    return stream_;
  }

private:
  cudaStream_t stream_{nullptr};
};

std::string cudaErrorMessage(const std::string & operation, const cudaError_t status)
{
  return operation + " failed: " + cudaGetErrorString(status);
}

std::string tensorRtDtypeToString(const nvinfer1::DataType dtype)
{
  if (dtype == nvinfer1::DataType::kFLOAT) {
    return "float32";
  }
  if (dtype == nvinfer1::DataType::kBOOL) {
    return "bool";
  }
  if (dtype == nvinfer1::DataType::kINT64) {
    return "int64";
  }
  if (dtype == nvinfer1::DataType::kINT32) {
    return "int32";
  }
  if (dtype == nvinfer1::DataType::kHALF) {
    return "float16";
  }
  return "unsupported";
}

std::vector<int64_t> tensorRtDimsToVector(const nvinfer1::Dims & dimensions)
{
  std::vector<int64_t> shape;
  if (dimensions.nbDims <= 0) {
    return shape;
  }
  shape.reserve(static_cast<std::size_t>(dimensions.nbDims));
  for (int32_t i = 0; i < dimensions.nbDims; ++i) {
    shape.push_back(dimensions.d[i]);
  }
  return shape;
}

bool checkedVolume(const nvinfer1::Dims & dimensions, std::size_t & volume)
{
  volume = 1;
  if (dimensions.nbDims <= 0) {
    return false;
  }
  for (int32_t i = 0; i < dimensions.nbDims; ++i) {
    if (dimensions.d[i] <= 0) {
      return false;
    }
    const auto dimension = static_cast<std::size_t>(dimensions.d[i]);
    if (volume > std::numeric_limits<std::size_t>::max() / dimension) {
      return false;
    }
    volume *= dimension;
  }
  return true;
}

std::string dimsToString(const nvinfer1::Dims & dimensions)
{
  std::ostringstream stream;
  stream << "[";
  for (int32_t i = 0; i < dimensions.nbDims; ++i) {
    if (i != 0) {
      stream << ", ";
    }
    stream << dimensions.d[i];
  }
  stream << "]";
  return stream.str();
}

struct HostInputView
{
  const void * data;
  std::size_t bytes;
};

class TensorRtMtrInferenceEngine final : public IMtrInferenceEngine
{
public:
  explicit TensorRtMtrInferenceEngine(MtrConfig config)
  : config_(std::move(config))
  {
    initialize();
  }

  bool ready() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return ready_;
  }

  std::string lastError() const override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_error_;
  }

  MtrOutputTensors infer(const MtrInputTensors & input) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    MtrOutputTensors output;

    if (!ready_ || context_ == nullptr) {
      if (last_error_.empty()) {
        setError("TensorRT backend is not ready");
      }
      return output;
    }
    if (!input.valid) {
      setError("MTR input tensors are marked invalid");
      return output;
    }

    std::vector<int64_t> track_indices;
    track_indices.reserve(input.track_index_to_predict.size());
    for (const int32_t track_index : input.track_index_to_predict) {
      track_indices.push_back(static_cast<int64_t>(track_index));
    }

    std::vector<int64_t> center_type_ids;
    if (!encodeCenterObjectTypes(input.center_objects_type, center_type_ids)) {
      return output;
    }
    if (!validateInput(input, track_indices, center_type_ids)) {
      return output;
    }

    const std::unordered_map<std::string, HostInputView> input_views{
      {"obj_trajs", {input.obj_trajs.data(), input.obj_trajs.size() * sizeof(float)}},
      {"obj_trajs_mask", {input.obj_trajs_mask.data(), input.obj_trajs_mask.size()}},
      {"obj_trajs_last_pos", {input.obj_trajs_last_pos.data(), input.obj_trajs_last_pos.size() * sizeof(float)}},
      {"map_polylines", {input.map_polylines.data(), input.map_polylines.size() * sizeof(float)}},
      {"map_polylines_mask", {input.map_polylines_mask.data(), input.map_polylines_mask.size()}},
      {"map_polylines_center", {input.map_polylines_center.data(), input.map_polylines_center.size() * sizeof(float)}},
      {"track_index_to_predict", {track_indices.data(), track_indices.size() * sizeof(int64_t)}},
      {"center_type_ids", {center_type_ids.data(), center_type_ids.size() * sizeof(int64_t)}}};

    std::unordered_map<std::string, DeviceBuffer> device_buffers;
    device_buffers.reserve(static_cast<std::size_t>(engine_->getNbIOTensors()));

    if (!prepareInputs(input_views, device_buffers)) {
      return output;
    }
    if (
      !prepareOutput("pred_scores", output.scores_shape, output.pred_scores, device_buffers) ||
      !prepareOutput("pred_trajs", output.trajs_shape, output.pred_trajs, device_buffers))
    {
      return output;
    }

    if (!context_->enqueueV3(stream_.get())) {
      setError("TensorRT enqueueV3 failed: " + logger_.lastMessage());
      return output;
    }
    if (
      !copyOutput("pred_scores", output.pred_scores, device_buffers) ||
      !copyOutput("pred_trajs", output.pred_trajs, device_buffers))
    {
      // enqueueV3 may still be using these buffers. Synchronize before their
      // RAII owners leave scope, while preserving the original copy error.
      (void)cudaStreamSynchronize(stream_.get());
      return MtrOutputTensors{};
    }

    const cudaError_t synchronize_status = cudaStreamSynchronize(stream_.get());
    if (synchronize_status != cudaSuccess) {
      setError(cudaErrorMessage("cudaStreamSynchronize", synchronize_status));
      return MtrOutputTensors{};
    }

    output.valid = true;
    last_error_.clear();
    return output;
  }

private:
  void setError(std::string error)
  {
    last_error_ = std::move(error);
  }

  void initialize()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    try {
      if (config_.engine_path.empty()) {
        setError("mtr.engine_path is empty");
        return;
      }

      expected_contract_ = loadMtrModelContract(config_.metadata_path);
      if (!loadSerializedEngine()) {
        return;
      }

      MtrModelContract actual_contract;
      if (!buildActualContract(actual_contract)) {
        return;
      }
      std::string contract_error;
      if (!validateMtrModelContract(expected_contract_, actual_contract, contract_error)) {
        setError("TensorRT contract validation failed: " + contract_error);
        return;
      }
      if (!validateEightTargetContract()) {
        return;
      }

      context_.reset(engine_->createExecutionContext());
      if (context_ == nullptr) {
        setError("failed to create TensorRT execution context: " + logger_.lastMessage());
        return;
      }

      const cudaError_t stream_status = stream_.create();
      if (stream_status != cudaSuccess) {
        setError(cudaErrorMessage("cudaStreamCreate", stream_status));
        return;
      }

      ready_ = true;
      last_error_.clear();
    } catch (const std::exception & exception) {
      setError(std::string("failed to initialize TensorRT backend: ") + exception.what());
    }
  }

  bool loadSerializedEngine()
  {
    std::ifstream file(config_.engine_path, std::ios::binary | std::ios::ate);
    if (!file) {
      setError("failed to open TensorRT engine: " + config_.engine_path);
      return false;
    }
    const std::streamsize file_size = file.tellg();
    if (file_size <= 0) {
      setError("TensorRT engine file is empty: " + config_.engine_path);
      return false;
    }
    file.seekg(0, std::ios::beg);
    std::vector<char> engine_bytes(static_cast<std::size_t>(file_size));
    if (!file.read(engine_bytes.data(), file_size)) {
      setError("failed to read TensorRT engine: " + config_.engine_path);
      return false;
    }

    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (runtime_ == nullptr) {
      setError("failed to create TensorRT runtime: " + logger_.lastMessage());
      return false;
    }
    engine_.reset(runtime_->deserializeCudaEngine(engine_bytes.data(), engine_bytes.size()));
    if (engine_ == nullptr) {
      setError("failed to deserialize TensorRT engine: " + logger_.lastMessage());
      return false;
    }
    return true;
  }

  bool buildActualContract(MtrModelContract & contract)
  {
    for (int32_t index = 0; index < engine_->getNbIOTensors(); ++index) {
      const char * name = engine_->getIOTensorName(index);
      if (name == nullptr) {
        setError("TensorRT engine returned a null I/O tensor name");
        return false;
      }

      MtrTensorSpec spec;
      spec.name = name;
      spec.dtype = tensorRtDtypeToString(engine_->getTensorDataType(name));
      spec.shape = tensorRtDimsToVector(engine_->getTensorShape(name));

      const auto io_mode = engine_->getTensorIOMode(name);
      if (io_mode == nvinfer1::TensorIOMode::kINPUT) {
        contract.inputs.push_back(std::move(spec));
      } else if (io_mode == nvinfer1::TensorIOMode::kOUTPUT) {
        contract.outputs.push_back(std::move(spec));
      } else {
        setError("TensorRT binding '" + std::string(name) + "' has an invalid I/O mode");
        return false;
      }
    }
    return true;
  }

  bool validateEightTargetContract()
  {
    const std::vector<int64_t> obj_shape{kTargetCapacity, kContextAgentCapacity, kHistorySteps, kObjectFeatureCount};
    const std::vector<int64_t> map_shape{kTargetCapacity, kMapPolylineCapacity, kPointsPerPolyline, kMapFeatureCount};

    if (tensorRtDimsToVector(engine_->getTensorShape("obj_trajs")) != obj_shape) {
      setError("obj_trajs does not use the fixed 8-target shape");
      return false;
    }
    if (tensorRtDimsToVector(engine_->getTensorShape("map_polylines")) != map_shape) {
      setError("map_polylines does not use the fixed 8-target shape");
      return false;
    }
    return true;
  }

  bool encodeCenterObjectTypes(const std::vector<std::string> & object_types, std::vector<int64_t> & type_ids)
  {
    type_ids.clear();
    type_ids.reserve(object_types.size());
    for (const auto & object_type : object_types) {
      if (object_type.empty() || object_type == "TYPE_VEHICLE") {
        type_ids.push_back(0);
      } else if (object_type == "TYPE_PEDESTRIAN") {
        type_ids.push_back(1);
      } else if (object_type == "TYPE_CYCLIST") {
        type_ids.push_back(2);
      } else {
        setError("unsupported center object type '" + object_type + "'");
        return false;
      }
    }
    return true;
  }

  template <typename T>
  bool requireElements(const std::string & name, const std::vector<T> & values, const std::size_t expected)
  {
    if (values.size() != expected) {
      setError(
        name + " element count mismatch: expected " + std::to_string(expected) + ", got " +
        std::to_string(values.size()));
      return false;
    }
    return true;
  }

  bool validateInput(
    const MtrInputTensors & input,
    const std::vector<int64_t> & track_indices,
    const std::vector<int64_t> & center_type_ids)
  {
    constexpr std::size_t target_capacity = static_cast<std::size_t>(kTargetCapacity);
    constexpr std::size_t context_capacity = static_cast<std::size_t>(kContextAgentCapacity);
    constexpr std::size_t history_steps = static_cast<std::size_t>(kHistorySteps);
    constexpr std::size_t object_features = static_cast<std::size_t>(kObjectFeatureCount);
    constexpr std::size_t map_capacity = static_cast<std::size_t>(kMapPolylineCapacity);
    constexpr std::size_t points_per_polyline = static_cast<std::size_t>(kPointsPerPolyline);
    constexpr std::size_t map_features = static_cast<std::size_t>(kMapFeatureCount);

    if (
      input.obj_trajs_shape !=
      std::vector<int64_t>{kTargetCapacity, kContextAgentCapacity, kHistorySteps, kObjectFeatureCount})
    {
      setError("obj_trajs_shape does not match [8, 128, 11, 29]");
      return false;
    }
    if (
      input.map_polylines_shape !=
      std::vector<int64_t>{kTargetCapacity, kMapPolylineCapacity, kPointsPerPolyline, kMapFeatureCount})
    {
      setError("map_polylines_shape does not match [8, 768, 20, 9]");
      return false;
    }
    if (input.sidecar.targets.size() > target_capacity) {
      setError("MTR sidecar contains more than 8 target agents");
      return false;
    }

    if (
      !requireElements(
        "obj_trajs", input.obj_trajs, target_capacity * context_capacity * history_steps * object_features) ||
      !requireElements("obj_trajs_mask", input.obj_trajs_mask, target_capacity * context_capacity * history_steps) ||
      !requireElements("obj_trajs_last_pos", input.obj_trajs_last_pos, target_capacity * context_capacity * 3) ||
      !requireElements(
        "map_polylines", input.map_polylines, target_capacity * map_capacity * points_per_polyline * map_features) ||
      !requireElements(
        "map_polylines_mask", input.map_polylines_mask, target_capacity * map_capacity * points_per_polyline) ||
      !requireElements("map_polylines_center", input.map_polylines_center, target_capacity * map_capacity * 3) ||
      !requireElements("track_index_to_predict", track_indices, target_capacity) ||
      !requireElements("center_type_ids", center_type_ids, target_capacity))
    {
      return false;
    }

    const auto is_boolean = [](const uint8_t value) { return value == 0 || value == 1; };
    if (!std::all_of(input.obj_trajs_mask.begin(), input.obj_trajs_mask.end(), is_boolean)) {
      setError("obj_trajs_mask must contain only 0 or 1");
      return false;
    }
    if (!std::all_of(input.map_polylines_mask.begin(), input.map_polylines_mask.end(), is_boolean)) {
      setError("map_polylines_mask must contain only 0 or 1");
      return false;
    }
    return true;
  }

  bool prepareInputs(
    const std::unordered_map<std::string, HostInputView> & input_views,
    std::unordered_map<std::string, DeviceBuffer> & device_buffers)
  {
    for (const auto & spec : expected_contract_.inputs) {
      const auto input = input_views.find(spec.name);
      if (input == input_views.end()) {
        setError("no host buffer is available for TensorRT input '" + spec.name + "'");
        return false;
      }

      DeviceBuffer buffer;
      const cudaError_t allocation_status = buffer.allocate(input->second.bytes);
      if (allocation_status != cudaSuccess) {
        setError(cudaErrorMessage("cudaMalloc(" + spec.name + ")", allocation_status));
        return false;
      }
      const cudaError_t copy_status =
        cudaMemcpyAsync(buffer.data(), input->second.data, input->second.bytes, cudaMemcpyHostToDevice, stream_.get());
      if (copy_status != cudaSuccess) {
        setError(cudaErrorMessage("cudaMemcpyAsync(" + spec.name + ")", copy_status));
        return false;
      }
      if (!context_->setTensorAddress(spec.name.c_str(), buffer.data())) {
        setError("failed to set TensorRT address for input '" + spec.name + "'");
        return false;
      }
      device_buffers.emplace(spec.name, std::move(buffer));
    }
    return true;
  }

  bool prepareOutput(
    const std::string & name,
    std::vector<int64_t> & shape,
    std::vector<float> & host_buffer,
    std::unordered_map<std::string, DeviceBuffer> & device_buffers)
  {
    const nvinfer1::Dims dimensions = context_->getTensorShape(name.c_str());
    std::size_t element_count = 0;
    if (!checkedVolume(dimensions, element_count)) {
      setError("invalid TensorRT output shape for '" + name + "': " + dimsToString(dimensions));
      return false;
    }

    shape = tensorRtDimsToVector(dimensions);
    host_buffer.resize(element_count);

    DeviceBuffer buffer;
    const cudaError_t allocation_status = buffer.allocate(element_count * sizeof(float));
    if (allocation_status != cudaSuccess) {
      setError(cudaErrorMessage("cudaMalloc(" + name + ")", allocation_status));
      return false;
    }
    if (!context_->setTensorAddress(name.c_str(), buffer.data())) {
      setError("failed to set TensorRT address for output '" + name + "'");
      return false;
    }
    device_buffers.emplace(name, std::move(buffer));
    return true;
  }

  bool copyOutput(
    const std::string & name,
    std::vector<float> & host_buffer,
    const std::unordered_map<std::string, DeviceBuffer> & device_buffers)
  {
    const auto device_buffer = device_buffers.find(name);
    if (device_buffer == device_buffers.end()) {
      setError("missing TensorRT device buffer for output '" + name + "'");
      return false;
    }
    const std::size_t bytes = host_buffer.size() * sizeof(float);
    if (device_buffer->second.bytes() != bytes) {
      setError("TensorRT output buffer size mismatch for '" + name + "'");
      return false;
    }

    const cudaError_t copy_status =
      cudaMemcpyAsync(host_buffer.data(), device_buffer->second.data(), bytes, cudaMemcpyDeviceToHost, stream_.get());
    if (copy_status != cudaSuccess) {
      setError(cudaErrorMessage("cudaMemcpyAsync(" + name + ")", copy_status));
      return false;
    }
    return true;
  }

  MtrConfig config_;
  mutable std::mutex mutex_;
  bool ready_{false};
  std::string last_error_;
  MtrModelContract expected_contract_;
  TensorRtLogger logger_;
  std::unique_ptr<nvinfer1::IRuntime> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext> context_;
  CudaStream stream_;
};

}  // namespace

std::unique_ptr<IMtrInferenceEngine> createTensorRtMtrInferenceEngine(const MtrConfig & config)
{
  return std::make_unique<TensorRtMtrInferenceEngine>(config);
}

}  // namespace detail
}  // namespace prediction_ml
