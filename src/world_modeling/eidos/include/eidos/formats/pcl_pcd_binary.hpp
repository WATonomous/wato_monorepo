#pragma once

#include <cstring>

#include <pcl/point_cloud.h>

#include "eidos/formats/format.hpp"
#include "eidos/utils/types.hpp"

namespace eidos::formats {

/**
 * @brief Format for pcl::PointCloud<PointType>::Ptr.
 *
 * Serializes as raw packed PointType array (no PCD header).
 * Fast memcpy-based read/write.
 */
class PclPcdBinary : public Format {
public:
  std::vector<uint8_t> serialize(const std::any& data) override {
    auto cloud = std::any_cast<pcl::PointCloud<PointType>::Ptr>(data);
    if (!cloud || cloud->empty()) return {};
    size_t total = cloud->size() * sizeof(PointType);
    std::vector<uint8_t> buf(total);
    std::memcpy(buf.data(), cloud->points.data(), total);
    return buf;
  }

  std::any deserialize(const std::vector<uint8_t>& bytes) override {
    if (bytes.empty()) return std::any{};
    size_t num = bytes.size() / sizeof(PointType);
    if (num == 0) return std::any{};
    auto cloud = pcl::make_shared<pcl::PointCloud<PointType>>();
    cloud->resize(num);
    std::memcpy(cloud->points.data(), bytes.data(), num * sizeof(PointType));
    cloud->width = static_cast<uint32_t>(num);
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
  }
};

}  // namespace eidos::formats
