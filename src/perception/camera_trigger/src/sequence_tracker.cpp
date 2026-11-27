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

#include "camera_trigger/sequence_tracker.hpp"

#include <algorithm>
#include <cstdint>

namespace camera_trigger
{

namespace
{

// Signed distance from `from` to `to` on the uint32 circle, in [-2^31, 2^31).
int64_t sequenceDistance(uint32_t from, uint32_t to)
{
  const uint32_t forward = to - from;  // wraps modulo 2^32
  if (forward < 0x80000000U) {
    return static_cast<int64_t>(forward);
  }
  return static_cast<int64_t>(forward) - (int64_t{1} << 32);
}

}  // namespace

bool SequenceResult::accepted() const
{
  return verdict != SequenceVerdict::kDuplicate && verdict != SequenceVerdict::kLate;
}

SequenceTracker::SequenceTracker(uint32_t reboot_threshold)
: reboot_threshold_(reboot_threshold)
{}

SequenceResult SequenceTracker::update(uint32_t seq)
{
  SequenceResult result;
  if (!has_last_) {
    result.verdict = SequenceVerdict::kFirst;
    accept(seq);
    return result;
  }

  const int64_t distance = sequenceDistance(last_, seq);
  if (distance == 1) {
    result.verdict = SequenceVerdict::kNext;
  } else if (distance > 1) {
    result.verdict = SequenceVerdict::kGap;
    result.missing = static_cast<uint32_t>(distance - 1);
  } else if (distance == 0) {
    result.verdict = SequenceVerdict::kDuplicate;
    return result;
  } else if (-distance <= static_cast<int64_t>(reboot_threshold_)) {
    result.verdict = recentlyAccepted(seq) ? SequenceVerdict::kDuplicate : SequenceVerdict::kLate;
    return result;
  } else {
    result.verdict = SequenceVerdict::kReboot;
    reset();
  }
  accept(seq);
  return result;
}

void SequenceTracker::reset()
{
  has_last_ = false;
  last_ = 0;
  recent_.clear();
}

void SequenceTracker::accept(uint32_t seq)
{
  has_last_ = true;
  last_ = seq;
  recent_.push_back(seq);
  if (recent_.size() > kRecentSize) {
    recent_.pop_front();
  }
}

bool SequenceTracker::recentlyAccepted(uint32_t seq) const
{
  return std::find(recent_.begin(), recent_.end(), seq) != recent_.end();
}

}  // namespace camera_trigger
