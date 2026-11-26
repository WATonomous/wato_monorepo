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

#include <functional>
#include <stdexcept>
#include <string>

namespace wato::perception::bevfusion
{
// Called only during initialization. CUDA failures propagate without changing the backend.
inline bool initializeLidarBackend(const std::string & requested, const std::function<void()> & initialize_cuda)
{
  if (requested == "cpu") return false;
  if (requested != "cuda") {
    throw std::invalid_argument("lidar_processing_backend must be cuda or cpu");
  }
  initialize_cuda();
  return true;
}
}  // namespace wato::perception::bevfusion
