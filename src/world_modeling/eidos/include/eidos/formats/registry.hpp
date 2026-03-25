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

#include <memory>
#include <string>
#include <unordered_map>

#include "eidos/formats/format.hpp"

namespace eidos::formats
{

/**
 * @brief Global registry of data formats.
 *
 * Maps format names (e.g. "pcl_pcd_binary") to Format instances.
 * Used by MapManager for persistence and by eidos_tools for export.
 *
 * Both eidos and eidos_tools link the same registry.
 */
const std::unordered_map<std::string, std::unique_ptr<Format>> & registry();

}  // namespace eidos::formats
