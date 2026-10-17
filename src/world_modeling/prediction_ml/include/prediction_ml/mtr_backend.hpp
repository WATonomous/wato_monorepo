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

#ifndef PREDICTION_ML__MTR_BACKEND_HPP_
#define PREDICTION_ML__MTR_BACKEND_HPP_

#include <string>

#include "prediction_ml/mtr_inference_engine.hpp"
#include "prediction_ml/mtr_types.hpp"

namespace prediction_ml
{

// Always-available engine that is never ready; forces fallback-only output.
class NullMtrInferenceEngine : public IMtrInferenceEngine
{
public:
  explicit NullMtrInferenceEngine(MtrConfig config);
  bool ready() const override;
  std::string lastError() const override;
  MtrOutputTensors infer(const MtrInputTensors & input) override;

private:
  MtrConfig config_;
  std::string last_error_{"mtr disabled (null backend)"};
};

}  // namespace prediction_ml

#endif  // PREDICTION_ML__MTR_BACKEND_HPP_
