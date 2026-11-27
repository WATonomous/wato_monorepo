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

#include <cstddef>
#include <cstdint>
#include <deque>

// Checks each pulse's sequence number against the ones before it. No ROS dependencies.

namespace camera_trigger
{

/// What the tracker decided about a pulse.
enum class SequenceVerdict
{
  kFirst,  ///< first pulse since start or reset: publish
  kNext,  ///< exactly the next pulse: publish
  kGap,  ///< later than expected, `missing` pulses were never reported: publish this one
  kDuplicate,  ///< same pulse as one already accepted: drop
  kLate,  ///< a little older than the last accepted pulse and never seen before (reordered): drop
  kReboot,  ///< far behind the last pulse: the board restarted. Tracking restarts here: publish
};

struct SequenceResult
{
  SequenceVerdict verdict{SequenceVerdict::kFirst};
  uint32_t missing{0};  ///< pulses skipped (kGap only)

  /// True if the pulse should be published.
  bool accepted() const;
};

/// Tracks pulse sequence numbers (uint32, +1 per pulse, wrapping at 2^32).
/// Never invents pulses: a gap is only counted, the missing pulses are not created.
class SequenceTracker
{
public:
  /// @param reboot_threshold a pulse more than this many behind the last one means the board rebooted
  explicit SequenceTracker(uint32_t reboot_threshold = 10);

  SequenceResult update(uint32_t seq);

  /// Forgets all history, e.g. after a BOOT line. The next pulse is kFirst.
  void reset();

  bool hasLast() const
  {
    return has_last_;
  }

  /// Last accepted sequence number (valid if hasLast()).
  uint32_t last() const
  {
    return last_;
  }

private:
  void accept(uint32_t seq);
  bool recentlyAccepted(uint32_t seq) const;

  static constexpr std::size_t kRecentSize = 64;

  uint32_t reboot_threshold_;
  bool has_last_{false};
  uint32_t last_{0};
  std::deque<uint32_t> recent_;  ///< recently accepted pulses, to tell duplicates from late pulses
};

}  // namespace camera_trigger
