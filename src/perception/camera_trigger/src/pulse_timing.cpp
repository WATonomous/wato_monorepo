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

#include "camera_trigger/pulse_timing.hpp"

#include <cstdint>
#include <stdexcept>

namespace camera_trigger
{

int64_t floorDiv(int64_t value, int64_t divisor)
{
  int64_t quotient = value / divisor;
  if (value % divisor != 0 && value < 0) {
    --quotient;
  }
  return quotient;
}

PulseTimer::PulseTimer(const PulseTimingConfig & config)
: config_(config)
{
  if (config_.pulses_per_second == 0 || config_.pulses_per_second > 256) {
    throw std::invalid_argument("pulses_per_second must be between 1 and 256");
  }
  if (config_.expected_latency_ns < 0 || config_.expected_latency_ns >= kNanosPerSecond / 2) {
    throw std::invalid_argument("expected_latency must be between 0 and 0.5 s");
  }
  if (config_.max_latency_ns <= 0) {
    throw std::invalid_argument("max_latency must be positive");
  }
  if (config_.offset_adopt_count == 0) {
    throw std::invalid_argument("offset_adopt_count must be at least 1");
  }
}

int64_t PulseTimer::pulseOffsetNs(uint32_t pulse_idx) const
{
  // Rounded to the nearest ns, so rates that don't divide 1 s (e.g. 30 Hz) also work.
  const int64_t pps = config_.pulses_per_second;
  return (static_cast<int64_t>(pulse_idx) * kNanosPerSecond + pps / 2) / pps;
}

PulseTime PulseTimer::compute(const TriggerReport & report, int64_t t_rx_ns)
{
  PulseTime result;
  result.locked = report.pps_locked;

  if (!report.pps_locked) {
    // Not on the PPS grid: best guess only, and it breaks any offset streak in progress.
    result.stamp_ns = t_rx_ns - config_.expected_latency_ns;
    candidate_count_ = 0;
    return result;
  }

  const int64_t pulse_offset = pulseOffsetNs(report.pulse_idx);
  const int64_t second =
    floorDiv(t_rx_ns - config_.expected_latency_ns - pulse_offset + kNanosPerSecond / 2, kNanosPerSecond);
  result.stamp_ns = second * kNanosPerSecond + pulse_offset;
  result.residual_ns = t_rx_ns - result.stamp_ns;
  result.residual_ok = result.residual_ns >= 0 && result.residual_ns <= config_.max_latency_ns;

  const int64_t offset = second - static_cast<int64_t>(report.pps_count);
  updateOffset(offset);
  result.offset_ok = established_ && offset == established_offset_;

  result.time_valid = result.residual_ok && result.offset_ok;
  return result;
}

void PulseTimer::reset()
{
  established_ = false;
  established_offset_ = 0;
  candidate_offset_ = 0;
  candidate_count_ = 0;
}

void PulseTimer::updateOffset(int64_t offset)
{
  if (established_ && offset == established_offset_) {
    candidate_count_ = 0;
    return;
  }
  // Disagrees with the established offset (or none yet): only adopt it after enough pulses agree.
  if (candidate_count_ > 0 && offset == candidate_offset_) {
    ++candidate_count_;
  } else {
    candidate_offset_ = offset;
    candidate_count_ = 1;
  }
  if (candidate_count_ >= config_.offset_adopt_count) {
    established_ = true;
    established_offset_ = offset;
    candidate_count_ = 0;
  }
}

}  // namespace camera_trigger
