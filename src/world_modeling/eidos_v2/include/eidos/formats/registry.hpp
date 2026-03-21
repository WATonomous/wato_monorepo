#pragma once

#include <memory>
#include <string>
#include <unordered_map>

#include "eidos/formats/format.hpp"

namespace eidos::formats {

/**
 * @brief Global registry of data formats.
 *
 * Maps format names (e.g. "pcl_pcd_binary") to Format instances.
 * Used by MapManager for persistence and by eidos_tools for export.
 *
 * Both eidos_v2 and eidos_tools link the same registry.
 */
const std::unordered_map<std::string, std::unique_ptr<Format>>& registry();

}  // namespace eidos::formats
