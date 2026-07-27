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

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "prediction_ml/bag_replay.hpp"
#include "prediction_ml/tensor_io.hpp"

namespace
{

std::vector<uint32_t> dims(const std::vector<int64_t> & shape)
{
  std::vector<uint32_t> result;
  result.reserve(shape.size());
  for (const int64_t dim : shape) {
    if (dim < 0 ||
      static_cast<uint64_t>(dim) > std::numeric_limits<uint32_t>::max())
    {
      throw std::runtime_error("SceneBuilder produced a dimension outside uint32");
    }
    result.push_back(static_cast<uint32_t>(dim));
  }
  return result;
}

prediction_ml::TensorBlob floatBlob(
  std::vector<uint32_t> shape, std::vector<float> values)
{
  prediction_ml::TensorBlob blob;
  blob.dtype = prediction_ml::TensorDType::Float32;
  blob.dims = std::move(shape);
  blob.f32 = std::move(values);
  return blob;
}

prediction_ml::TensorBlob maskBlob(
  std::vector<uint32_t> shape, std::vector<uint8_t> values)
{
  prediction_ml::TensorBlob blob;
  blob.dtype = prediction_ml::TensorDType::UInt8;
  blob.dims = std::move(shape);
  blob.u8 = std::move(values);
  return blob;
}

prediction_ml::TensorBlob integerBlob(
  std::vector<uint32_t> shape, std::vector<int64_t> values)
{
  prediction_ml::TensorBlob blob;
  blob.dtype = prediction_ml::TensorDType::Int64;
  blob.dims = std::move(shape);
  blob.i64 = std::move(values);
  return blob;
}

prediction_ml::TensorBundle toBundle(prediction_ml::MtrInputTensors tensors)
{
  const auto object_shape = dims(tensors.obj_trajs_shape);
  const auto map_shape = dims(tensors.map_polylines_shape);
  if (object_shape.size() != 4U || map_shape.size() != 4U) {
    throw std::runtime_error("SceneBuilder produced non-contract input ranks");
  }

  prediction_ml::TensorBundle bundle;
  bundle.emplace(
    "obj_trajs", floatBlob(object_shape, std::move(tensors.obj_trajs)));
  bundle.emplace(
    "obj_trajs_mask",
    maskBlob(
      {object_shape[0], object_shape[1], object_shape[2]},
      std::move(tensors.obj_trajs_mask)));
  bundle.emplace(
    "obj_trajs_last_pos",
    floatBlob(
      {object_shape[0], object_shape[1], 3U},
      std::move(tensors.obj_trajs_last_pos)));
  bundle.emplace(
    "track_index_to_predict",
    integerBlob({object_shape[0]}, std::move(tensors.track_index_to_predict)));
  bundle.emplace(
    "center_type_ids",
    integerBlob({object_shape[0]}, std::move(tensors.center_type_ids)));
  bundle.emplace(
    "map_polylines", floatBlob(map_shape, std::move(tensors.map_polylines)));
  bundle.emplace(
    "map_polylines_mask",
    maskBlob(
      {map_shape[0], map_shape[1], map_shape[2]},
      std::move(tensors.map_polylines_mask)));
  bundle.emplace(
    "map_polylines_center",
    floatBlob(
      {map_shape[0], map_shape[1], 3U},
      std::move(tensors.map_polylines_center)));
  return bundle;
}

std::vector<std::string> parsePinned(const std::string & value)
{
  std::vector<std::string> ids;
  std::size_t begin = 0U;
  while (begin <= value.size()) {
    const std::size_t end = value.find(',', begin);
    auto id = prediction_ml::bag_replay::trim(
      value.substr(begin, end == std::string::npos ? end : end - begin));
    if (id.empty()) {
      throw std::invalid_argument("--pinned contains an empty target id");
    }
    ids.push_back(std::move(id));
    if (end == std::string::npos) {
      break;
    }
    begin = end + 1U;
  }
  if (ids.empty()) {
    throw std::invalid_argument("--pinned requires at least one target id");
  }
  if (ids.size() > prediction_ml::bag_replay::kTargetCapacity) {
    ids.resize(prediction_ml::bag_replay::kTargetCapacity);
  }
  return ids;
}

void usage(const char * executable)
{
  std::cerr << "Usage: " << executable
            << " <bag_dir> <out.bin.gz> [--pinned id1,id2,...]\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  if (argc != 3 && argc != 5) {
    usage(argv[0]);
    return 2;
  }

  try {
    const std::filesystem::path bag_dir{argv[1]};
    std::vector<std::string> pinned_ids;
    if (argc == 5) {
      if (std::string{argv[3]} != "--pinned") {
        usage(argv[0]);
        return 2;
      }
      pinned_ids = parsePinned(argv[4]);
    } else {
      pinned_ids = prediction_ml::bag_replay::readPinnedTargetIds(bag_dir);
    }

    auto tensors = prediction_ml::bag_replay::buildFromBag(
      bag_dir, pinned_ids, prediction_ml::bag_replay::readReplayStep(bag_dir));
    if (!tensors.valid) {
      throw std::runtime_error("SceneBuilder produced no valid tensors");
    }
    prediction_ml::writeTensorBundle(argv[2], toBundle(std::move(tensors)));
    std::cout << "wrote " << argv[2] << '\n';
    return 0;
  } catch (const std::exception & error) {
    std::cerr << "scene_dump: " << error.what() << '\n';
    return 1;
  }
}
