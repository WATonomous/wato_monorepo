#pragma once

#include <cstring>

#include <gtsam/geometry/Point3.h>

#include "eidos/formats/format.hpp"

namespace eidos::formats {

/**
 * @brief Format for gtsam::Point3 (Eigen::Vector3d).
 * Serializes as 3 packed doubles (24 bytes).
 */
class RawDouble3 : public Format {
public:
  std::vector<uint8_t> serialize(const std::any& data) override {
    auto pt = std::any_cast<gtsam::Point3>(data);
    std::vector<uint8_t> buf(3 * sizeof(double));
    double vals[3] = {pt.x(), pt.y(), pt.z()};
    std::memcpy(buf.data(), vals, buf.size());
    return buf;
  }

  std::any deserialize(const std::vector<uint8_t>& bytes) override {
    if (bytes.size() < 3 * sizeof(double)) return std::any{};
    double vals[3];
    std::memcpy(vals, bytes.data(), 3 * sizeof(double));
    return gtsam::Point3(vals[0], vals[1], vals[2]);
  }
};

}  // namespace eidos::formats
