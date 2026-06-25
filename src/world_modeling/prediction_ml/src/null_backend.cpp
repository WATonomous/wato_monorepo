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

#include <algorithm>
#include <cctype>
#include <memory>
#include <string>
#include <utility>

#include "prediction_ml/mtr_backend.hpp"

namespace prediction_ml
{

NullMtrInferenceEngine::NullMtrInferenceEngine(MtrConfig config)
: config_(std::move(config))
{}

bool NullMtrInferenceEngine::ready() const
{
  return false;
}

std::string NullMtrInferenceEngine::lastError() const
{
  return last_error_;
}

MtrOutputTensors NullMtrInferenceEngine::infer(const MtrInputTensors & /*input*/)
{
  MtrOutputTensors out;
  out.valid = false;
  return out;
}

std::unique_ptr<IMtrInferenceEngine> createNullMtrInferenceEngine(const MtrConfig & config)
{
  return std::make_unique<NullMtrInferenceEngine>(config);
}

#ifndef PREDICTION_ML_ENABLE_TENSORRT
std::unique_ptr<IMtrInferenceEngine> createTensorRtMtrInferenceEngine(const MtrConfig & config)
{
  // TensorRT not compiled in: behave exactly like the null backend.
  return std::make_unique<NullMtrInferenceEngine>(config);
}
#endif

MtrMode parseMtrMode(const std::string & mode_str)
{
  std::string s = mode_str;
  std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c) { return std::tolower(c); });
  if (s == "null") {
    return MtrMode::Null;
  }
  if (s == "tensorrt" || s == "trt") {
    return MtrMode::TensorRt;
  }
  return MtrMode::Disabled;
}

MtrModelContract loadMtrModelContract(const std::string & /*metadata_path*/)
{
  // TODO(Person B): parse metadata file into expected contract.
  return MtrModelContract{};
}

bool validateMtrModelContract(
  const MtrModelContract & /*expected*/, const MtrModelContract & /*actual*/, std::string & error)
{
  // TODO(Person B): compare binding names/dtypes/shapes.
  error.clear();
  return true;
}

}  // namespace prediction_ml
