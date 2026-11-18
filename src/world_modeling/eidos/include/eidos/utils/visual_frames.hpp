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

#include <gtsam/geometry/Pose3.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <deque>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace eidos
{

/// One compressed camera image (ROS-free copy of sensor_msgs/CompressedImage).
struct VisualFrame
{
  std::string frame_id;  ///< Camera optical frame
  double stamp = 0.0;  ///< s
  std::string format;  ///< As in sensor_msgs/CompressedImage, e.g. "bgr8; jpeg compressed bgr8"
  std::vector<uint8_t> data;
};

/// The images of all cameras for one instant. Stored per keyframe by the visual plugins.
struct VisualFrameSet
{
  double stamp = 0.0;  ///< Stamp of the first camera's image (s)
  std::vector<std::shared_ptr<const VisualFrame>> frames;  ///< One per camera, in camera order
};

/**
 * @brief Short per-camera history of compressed images (ROS-free, not thread-safe).
 *
 * Cameras are not synchronized with each other or with the keyframe sensor, so a frame set for a
 * time t is every camera's frame nearest to t.
 */
class VisualFrameBuffer
{
public:
  VisualFrameBuffer(std::size_t num_cameras, double duration)
  : buffers_(num_cameras)
  , duration_(duration)
  {}

  /// @brief Append a frame of one camera. Frames older than the camera's newest are ignored.
  void push(std::size_t camera, std::shared_ptr<const VisualFrame> frame)
  {
    auto & buf = buffers_.at(camera);
    if (!buf.empty() && frame->stamp <= buf.back()->stamp) return;
    buf.push_back(std::move(frame));
    while (buf.size() > 1 && buf.back()->stamp - buf.front()->stamp > duration_) buf.pop_front();
  }

  void clear()
  {
    for (auto & buf : buffers_) buf.clear();
  }

  /// @brief Whether every camera has delivered a frame at or after t (its nearest frame to t is then known).
  bool covers(double t) const
  {
    return std::all_of(
      buffers_.begin(), buffers_.end(), [t](const auto & buf) { return !buf.empty() && buf.back()->stamp >= t; });
  }

  /// @brief Newest stamp over all cameras, or nullopt if no frame has arrived.
  std::optional<double> newest() const
  {
    std::optional<double> out;
    for (const auto & buf : buffers_) {
      if (!buf.empty() && (!out || buf.back()->stamp > *out)) out = buf.back()->stamp;
    }
    return out;
  }

  /// @brief Newest stamp that every camera has reached, or nullopt if a camera has no frame.
  std::optional<double> newestCommon() const
  {
    std::optional<double> out;
    for (const auto & buf : buffers_) {
      if (buf.empty()) return std::nullopt;
      if (!out || buf.back()->stamp < *out) out = buf.back()->stamp;
    }
    return out;
  }

  /**
   * @brief Every camera's frame nearest to t.
   * @param t Time to select around (s).
   * @param tolerance Largest accepted |frame stamp - t| (s).
   * @return The frame set, or nullopt if a camera has no frame within the tolerance.
   */
  std::optional<VisualFrameSet> nearest(double t, double tolerance) const
  {
    VisualFrameSet set;
    for (const auto & buf : buffers_) {
      std::shared_ptr<const VisualFrame> best;
      for (const auto & frame : buf) {
        if (!best || std::abs(frame->stamp - t) < std::abs(best->stamp - t)) best = frame;
      }
      if (!best || std::abs(best->stamp - t) > tolerance) return std::nullopt;
      set.frames.push_back(best);
    }
    if (set.frames.empty()) return std::nullopt;
    set.stamp = set.frames.front()->stamp;
    return set;
  }

private:
  std::vector<std::deque<std::shared_ptr<const VisualFrame>>> buffers_;
  double duration_;
};

/**
 * @brief Motion over dt of a body moving at the constant twist that takes pose a (at ta) to pose b (at tb).
 * @return T(t)^-1 * T(t + dt) for any t; identity if the two poses are not at distinct times.
 */
inline gtsam::Pose3 constantVelocityMotion(
  const gtsam::Pose3 & a, double ta, const gtsam::Pose3 & b, double tb, double dt)
{
  if (std::abs(tb - ta) < 1e-6) return gtsam::Pose3();
  const gtsam::Vector6 twist = gtsam::Pose3::Logmap(a.between(b)) / (tb - ta);
  return gtsam::Pose3::Expmap(twist * dt);
}

/**
 * @brief How alike two places look, from the place descriptors of their camera images.
 *
 * Each descriptor set holds one L2-normalised descriptor of length dim per camera, concatenated.
 * Every camera of a is compared with its best match in b (the vehicle may face another way), and
 * the cameras' scores are averaged.
 *
 * @return Mean best cosine similarity in [-1, 1], or -1 if either set is empty or malformed.
 */
inline double placeSimilarity(const std::vector<float> & a, const std::vector<float> & b, std::size_t dim)
{
  if (dim == 0 || a.empty() || b.empty() || a.size() % dim != 0 || b.size() % dim != 0) return -1.0;
  const std::size_t na = a.size() / dim, nb = b.size() / dim;
  double total = 0.0;
  for (std::size_t i = 0; i < na; ++i) {
    double best = -1.0;
    for (std::size_t j = 0; j < nb; ++j) {
      double dot = 0.0;
      const float * pa = a.data() + i * dim;
      const float * pb = b.data() + j * dim;
      for (std::size_t k = 0; k < dim; ++k) dot += static_cast<double>(pa[k]) * static_cast<double>(pb[k]);
      best = std::max(best, dot);
    }
    total += best;
  }
  return total / static_cast<double>(na);
}

}  // namespace eidos
