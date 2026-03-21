#pragma once

#include <array>
#include <cstring>

#include "eidos/formats/format.hpp"

namespace eidos::formats {

/**
 * @brief Format for std::array<double, 5>.
 * Serializes as 5 packed doubles (40 bytes).
 */
class RawDouble5 : public Format {
public:
  std::vector<uint8_t> serialize(const std::any& data) override {
    auto arr = std::any_cast<std::array<double, 5>>(data);
    std::vector<uint8_t> buf(5 * sizeof(double));
    std::memcpy(buf.data(), arr.data(), buf.size());
    return buf;
  }

  std::any deserialize(const std::vector<uint8_t>& bytes) override {
    if (bytes.size() < 5 * sizeof(double)) return std::any{};
    std::array<double, 5> arr;
    std::memcpy(arr.data(), bytes.data(), 5 * sizeof(double));
    return arr;
  }
};

}  // namespace eidos::formats
