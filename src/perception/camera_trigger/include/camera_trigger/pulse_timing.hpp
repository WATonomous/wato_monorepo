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

#include <cstdint>

#include "camera_trigger/protocol.hpp"

// Turns a pulse report plus its receive time into the time the pulse fired. No ROS dependencies.
//
// PPS second boundaries are the same in GPS time and UTC, so once the system clock is synced to
// GPS (chrony, issue #560) a locked pulse fired at exactly
//     second + pulse_idx / pulses_per_second
// The second is found by rounding (receive time - expected latency - pulse_idx * period).
// That is exact as long as the real delay stays within about half a second of expected_latency.

namespace camera_trigger
{

inline constexpr int64_t kNanosPerSecond = 1000000000;

struct PulseTimingConfig
{
  uint32_t pulses_per_second{20};
  int64_t expected_latency_ns{10000000};  ///< typical pulse-to-receive delay
  int64_t max_latency_ns{100000000};  ///< larger delay means the clock isn't synced, or a stale report
  uint32_t offset_adopt_count{20};  ///< consecutive agreeing pulses needed to (re)establish the second offset
};

struct PulseTime
{
  int64_t stamp_ns{0};  ///< when the pulse fired, on the system clock
  bool time_valid{false};  ///< locked, residual within limits, and second counter consistent
  bool locked{false};  ///< the report said the pulse train is locked to PPS
  bool residual_ok{false};  ///< 0 <= residual <= max latency (locked pulses only)
  bool offset_ok{false};  ///< the second counter agrees with the established offset (locked pulses only)
  int64_t residual_ns{0};  ///< receive time - pulse time (locked pulses only)
};

/// Computes pulse times. Keeps the offset between the board's second counter and the clock's
/// seconds, so a single report that disagrees with the others is marked invalid.
class PulseTimer
{
public:
  /// @throws std::invalid_argument if the configuration is unusable
  explicit PulseTimer(const PulseTimingConfig & config);

  /// @param t_rx_ns system-clock time the report was received
  PulseTime compute(const TriggerReport & report, int64_t t_rx_ns);

  /// Forgets the second offset (board rebooted). The next offset_adopt_count pulses are invalid.
  void reset();

  bool offsetEstablished() const
  {
    return established_;
  }

  /// Clock second minus the board's pps_count (valid if offsetEstablished()).
  int64_t offset() const
  {
    return established_offset_;
  }

  /// Time of pulse `pulse_idx` after the start of its second, in ns.
  int64_t pulseOffsetNs(uint32_t pulse_idx) const;

private:
  void updateOffset(int64_t offset);

  PulseTimingConfig config_;
  bool established_{false};
  int64_t established_offset_{0};
  int64_t candidate_offset_{0};
  uint32_t candidate_count_{0};
};

/// Floor division (rounds toward negative infinity). `divisor` must be positive.
int64_t floorDiv(int64_t value, int64_t divisor);

}  // namespace camera_trigger
