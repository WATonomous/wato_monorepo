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

#include "eidos/formats/pcl_pcd_binary.hpp"
#include "eidos/formats/raw_double3.hpp"
#include "eidos/formats/raw_double4_eigen.hpp"
#include "eidos/formats/raw_double5.hpp"
#include "eidos/formats/registry.hpp"
#include "eidos/formats/small_gicp_binary.hpp"

namespace eidos::formats
{

const std::unordered_map<std::string, std::unique_ptr<Format>> & registry()
{
  static auto reg = [] {
    std::unordered_map<std::string, std::unique_ptr<Format>> r;
    r["pcl_pcd_binary"] = std::make_unique<PclPcdBinary>();
    r["raw_double3"] = std::make_unique<RawDouble3>();
    r["raw_double5"] = std::make_unique<RawDouble5>();
    r["raw_double4_eigen"] = std::make_unique<RawDouble4Eigen>();
    r["small_gicp_binary"] = std::make_unique<SmallGicpBinary>();
    return r;
  }();
  return reg;
}

}  // namespace eidos::formats
