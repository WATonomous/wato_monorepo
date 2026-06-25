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

#include <memory>

#include "prediction_ml/mtr_backend.hpp"

// This translation unit is compiled ONLY when PREDICTION_ML_ENABLE_TENSORRT is set.
namespace prediction_ml
{

std::unique_ptr<IMtrInferenceEngine> createTensorRtMtrInferenceEngine(const MtrConfig & config)
{
  // TODO(Person B): load engine_path, validate bindings against MtrModelContract,
  // implement infer(). For the skeleton, fall back to the null engine so a TRT-enabled
  // build still links and runs.
  return createNullMtrInferenceEngine(config);
}

}  // namespace prediction_ml
