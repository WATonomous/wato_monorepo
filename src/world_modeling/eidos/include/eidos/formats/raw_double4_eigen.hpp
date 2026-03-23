#pragma once

#include <cstring>

#include <Eigen/Core>

#include "eidos/formats/format.hpp"

namespace eidos::formats {

/**
 * @brief Format for Eigen::Vector4d.
 * Serializes as 4 packed doubles (32 bytes).
 */
class RawDouble4Eigen : public Format {
public:
  std::vector<uint8_t> serialize(const std::any& data) override {
    auto vec = std::any_cast<Eigen::Vector4d>(data);
    std::vector<uint8_t> buf(4 * sizeof(double));
    double vals[4] = {vec[0], vec[1], vec[2], vec[3]};
    std::memcpy(buf.data(), vals, buf.size());
    return buf;
  }

  std::any deserialize(const std::vector<uint8_t>& bytes) override {
    if (bytes.size() < 4 * sizeof(double)) return std::any{};
    double vals[4];
    std::memcpy(vals, bytes.data(), 4 * sizeof(double));
    return Eigen::Vector4d(vals[0], vals[1], vals[2], vals[3]);
  }
};

}  // namespace eidos::formats
