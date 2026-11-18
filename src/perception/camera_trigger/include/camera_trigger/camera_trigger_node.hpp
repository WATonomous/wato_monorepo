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

#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>

#include <camera_trigger_msgs/msg/camera_trigger.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <rclcpp/rclcpp.hpp>

#include "camera_trigger/protocol.hpp"
#include "camera_trigger/pulse_timing.hpp"
#include "camera_trigger/sequence_tracker.hpp"
#include "camera_trigger/transport.hpp"

namespace camera_trigger
{

/**
 * @brief Publishes each hardware trigger pulse with the exact time it fired.
 *
 * The STM32 trigger board fires a PPS-locked pulse into the cameras and then reports it
 * (one UDP datagram per pulse). This node turns each report into a CameraTrigger message on
 * /camera_trigger, stamped with the pulse time on the system clock, and publishes its health on
 * /diagnostics at 2 Hz.
 *
 * Reports are read on a dedicated thread, so network problems never block the executor, and the
 * node is launched as its own process so a fault here cannot stop the camera drivers.
 */
class CameraTriggerNode : public rclcpp::Node
{
public:
  explicit CameraTriggerNode(const rclcpp::NodeOptions & options);
  ~CameraTriggerNode() override;

private:
  using SteadyClock = std::chrono::steady_clock;
  using SteadyTime = SteadyClock::time_point;

  static constexpr std::size_t kNumStrobes = 12;
  static constexpr std::size_t kNumParseErrors = 7;

  /// Everything the diagnostics report. Written by the I/O thread, read by the diagnostics timer.
  struct Stats
  {
    // Link
    bool link_open{false};
    std::string link_error;
    std::string endpoint;
    std::string last_source;
    bool kernel_timestamps{true};

    // Board
    std::string fw_version{"unknown"};

    // Totals since start
    uint64_t reports_received{0};
    uint64_t pulses_published{0};
    uint64_t gaps{0};
    uint64_t missing_pulses{0};
    uint64_t duplicates{0};
    uint64_t late{0};
    uint64_t board_starts{0};  ///< BOOT lines, plus restarts inferred from the sequence jumping back
    uint64_t bad_lines{0};
    std::array<uint64_t, kNumParseErrors> bad_lines_by_type{};
    uint64_t unknown_lines{0};
    uint64_t foreign_packets{0};
    uint64_t invalid_stamps{0};

    // Latest pulse
    bool have_pulse{false};
    SteadyTime last_pulse_time{};
    TriggerReport last_report;
    PulseTime last_time;
    bool offset_established{false};
    bool have_residual{false};
    int64_t last_residual_ns{0};

    // Recent history
    bool receiving{false};
    SteadyTime receiving_since{};
    std::deque<SteadyTime> pulse_times;  // last 2 s
    std::deque<std::pair<SteadyTime, uint32_t>> missing_events;  // last 10 s
    std::deque<SteadyTime> bad_line_times;  // last 10 s
    bool residual_failed{false};
    SteadyTime last_residual_failure{};
    int64_t last_bad_residual_ns{0};

    // Strobes: consecutive pulses without a strobe, per input STB1..STB12
    bool strobes_measured{false};
    std::array<uint32_t, kNumStrobes> strobe_misses{};
  };

  void ioLoop();
  void handleLine(const ReceivedLine & line);
  void handleTrigger(const TriggerReport & report, const ReceivedLine & line);
  void handleBoot(const BootReport & boot, const ReceivedLine & line);
  void countBadLine(ParseError error, const std::string & source);
  void publishDiagnostics();
  void pruneHistory(SteadyTime now);
  void sleepFor(std::chrono::milliseconds duration);

  // Parameters
  uint32_t pulses_per_second_{20};
  double max_latency_s_{0.1};
  std::chrono::nanoseconds stale_timeout_{};
  std::chrono::milliseconds reconnect_period_{};
  uint32_t strobe_miss_warn_count_{20};
  uint16_t expected_strobe_mask_{0x0FFF};
  std::string source_ip_;

  // Used only by the I/O thread
  std::unique_ptr<Transport> transport_;
  std::unique_ptr<PulseTimer> timer_;
  SequenceTracker sequence_;
  bool logged_no_kernel_timestamp_{false};

  rclcpp::Publisher<camera_trigger_msgs::msg::CameraTrigger>::SharedPtr trigger_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diagnostics_pub_;
  rclcpp::TimerBase::SharedPtr diagnostics_timer_;

  std::mutex mutex_;
  Stats stats_;  // guarded by mutex_

  std::atomic<bool> stop_{false};
  std::thread io_thread_;  // last member: started after everything above is ready
};

}  // namespace camera_trigger
