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

#include "camera_trigger/camera_trigger_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <variant>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp_components/register_node_macro.hpp>

namespace camera_trigger
{

namespace
{

using diagnostic_msgs::msg::DiagnosticStatus;

constexpr char kFrameId[] = "camera_trigger";
constexpr char kHardwareId[] = "stm32_trigger_board";
constexpr int kReadTimeoutMs = 100;
constexpr std::chrono::milliseconds kMinRetryDelay{10};
constexpr std::chrono::milliseconds kMaxRetryDelay{10000};
constexpr std::chrono::milliseconds kDiagnosticsPeriod{500};
constexpr std::chrono::seconds kRateWindow{2};
constexpr std::chrono::seconds kEventWindow{10};
constexpr std::chrono::seconds kResidualWindow{1};
constexpr double kRateToleranceHz = 0.5;

builtin_interfaces::msg::Time toRosTime(int64_t ns)
{
  builtin_interfaces::msg::Time time;
  const int64_t sec = floorDiv(ns, kNanosPerSecond);
  time.sec = static_cast<int32_t>(sec);
  time.nanosec = static_cast<uint32_t>(ns - sec * kNanosPerSecond);
  return time;
}

std::string fixed(double value, int decimals)
{
  char buffer[64];
  std::snprintf(buffer, sizeof(buffer), "%.*f", decimals, value);
  return buffer;
}

std::string hex3(uint16_t value)
{
  char buffer[8];
  std::snprintf(buffer, sizeof(buffer), "%03X", static_cast<unsigned>(value));
  return buffer;
}

double toMs(int64_t ns)
{
  return static_cast<double>(ns) / 1e6;
}

std::string strobeName(std::size_t bit)
{
  return "STB" + std::to_string(bit + 1);
}

std::string joined(const std::vector<std::string> & parts, const std::string & separator)
{
  std::string text;
  for (std::size_t i = 0; i < parts.size(); ++i) {
    if (i > 0) {
      text += separator;
    }
    text += parts[i];
  }
  return text;
}

}  // namespace

CameraTriggerNode::CameraTriggerNode(const rclcpp::NodeOptions & options)
: Node("camera_trigger", options)
{
  const auto bind_address = declare_parameter<std::string>("udp_bind_address", "0.0.0.0");
  const auto udp_port = declare_parameter<int64_t>("udp_port", 5800);
  source_ip_ = declare_parameter<std::string>("udp_source_ip", "");
  const auto pulses_per_second = declare_parameter<int64_t>("pulses_per_second", 20);
  const auto expected_latency = declare_parameter<double>("expected_latency", 0.010);
  max_latency_s_ = declare_parameter<double>("max_latency", 0.100);
  const auto stale_timeout = declare_parameter<double>("stale_timeout", 0.3);
  const auto reconnect_period = declare_parameter<double>("reconnect_period", 1.0);
  const auto strobe_miss_warn_count = declare_parameter<int64_t>("strobe_miss_warn_count", 20);
  const auto expected_strobe_bits = declare_parameter<std::vector<int64_t>>(
    "expected_strobe_bits", std::vector<int64_t>{0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11});

  // Bad configuration fails at startup with a clear message instead of retrying forever.
  if (!isValidIpv4(bind_address)) {
    throw std::invalid_argument("udp_bind_address must be an IPv4 address, got '" + bind_address + "'");
  }
  if (!source_ip_.empty() && !isValidIpv4(source_ip_)) {
    throw std::invalid_argument("udp_source_ip must be empty or an IPv4 address, got '" + source_ip_ + "'");
  }
  if (udp_port < 0 || udp_port > 65535) {
    throw std::invalid_argument("udp_port must be between 0 and 65535");
  }
  if (pulses_per_second < 1 || pulses_per_second > 256) {
    throw std::invalid_argument("pulses_per_second must be between 1 and 256");
  }
  if (stale_timeout <= 0.0 || reconnect_period <= 0.0) {
    throw std::invalid_argument("stale_timeout and reconnect_period must be positive");
  }
  if (strobe_miss_warn_count < 1) {
    throw std::invalid_argument("strobe_miss_warn_count must be at least 1");
  }
  expected_strobe_mask_ = 0;
  for (const int64_t bit : expected_strobe_bits) {
    if (bit < 0 || bit >= static_cast<int64_t>(kNumStrobes)) {
      throw std::invalid_argument("expected_strobe_bits must be between 0 (STB1) and 11 (STB12)");
    }
    expected_strobe_mask_ = static_cast<uint16_t>(expected_strobe_mask_ | (1U << bit));
  }

  pulses_per_second_ = static_cast<uint32_t>(pulses_per_second);
  strobe_miss_warn_count_ = static_cast<uint32_t>(strobe_miss_warn_count);
  stale_timeout_ = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(stale_timeout));
  reconnect_period_ = std::max(
    kMinRetryDelay,
    std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(reconnect_period)));

  PulseTimingConfig timing;
  timing.pulses_per_second = pulses_per_second_;
  timing.expected_latency_ns = std::llround(expected_latency * 1e9);
  timing.max_latency_ns = std::llround(max_latency_s_ * 1e9);
  timer_ = std::make_unique<PulseTimer>(timing);  // throws std::invalid_argument on bad values

  UdpConfig udp;
  udp.bind_address = bind_address;
  udp.port = static_cast<uint16_t>(udp_port);
  udp.source_ip = source_ip_;
  transport_ = std::make_unique<UdpTransport>(udp);
  stats_.endpoint = transport_->describe();

  trigger_pub_ = create_publisher<camera_trigger_msgs::msg::CameraTrigger>(
    "/camera_trigger", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
  diagnostics_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", rclcpp::QoS(10));
  diagnostics_timer_ = create_wall_timer(kDiagnosticsPeriod, [this]() { publishDiagnostics(); });

  RCLCPP_INFO(
    get_logger(),
    "Expecting %u pulses/s on %s (sender %s), latency %.1f ms expected / %.1f ms max",
    pulses_per_second_,
    stats_.endpoint.c_str(),
    source_ip_.empty() ? "any" : source_ip_.c_str(),
    expected_latency * 1e3,
    max_latency_s_ * 1e3);

  io_thread_ = std::thread([this]() { ioLoop(); });
}

CameraTriggerNode::~CameraTriggerNode()
{
  stop_ = true;
  if (io_thread_.joinable()) {
    io_thread_.join();
  }
  if (transport_) {
    transport_->close();
  }
}

void CameraTriggerNode::sleepFor(std::chrono::milliseconds duration)
{
  const SteadyTime deadline = SteadyClock::now() + duration;
  while (!stop_ && SteadyClock::now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
}

void CameraTriggerNode::ioLoop()
{
  std::chrono::milliseconds retry_delay = reconnect_period_;
  bool logged_open_failure = false;

  while (!stop_) {
    if (!transport_->isOpen()) {
      std::string error;
      if (transport_->open(error)) {
        RCLCPP_INFO(get_logger(), "Listening for trigger board reports on %s", transport_->describe().c_str());
        std::lock_guard<std::mutex> lock(mutex_);
        stats_.link_open = true;
        stats_.link_error.clear();
        stats_.endpoint = transport_->describe();
        retry_delay = reconnect_period_;
        logged_open_failure = false;
      } else {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stats_.link_open = false;
          stats_.link_error = error;
        }
        if (!logged_open_failure) {
          RCLCPP_ERROR(
            get_logger(), "Cannot listen on %s: %s. Retrying.", transport_->describe().c_str(), error.c_str());
          logged_open_failure = true;
        } else {
          RCLCPP_ERROR_THROTTLE(
            get_logger(),
            *get_clock(),
            30000,
            "Still cannot listen on %s: %s",
            transport_->describe().c_str(),
            error.c_str());
        }
        sleepFor(retry_delay);
        retry_delay = std::min(retry_delay * 2, kMaxRetryDelay);
        continue;
      }
    }

    ReceivedLine line;
    switch (transport_->read(line, kReadTimeoutMs)) {
      case ReadStatus::kLine:
        handleLine(line);
        break;
      case ReadStatus::kTimeout:
        break;
      case ReadStatus::kTooLong:
        countBadLine(ParseError::kTooLong, line.source);
        break;
      case ReadStatus::kForeignSender: {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          ++stats_.foreign_packets;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          10000,
          "Ignoring packet from %s (only %s is accepted)",
          line.source.c_str(),
          source_ip_.c_str());
        break;
      }
      case ReadStatus::kError: {
        RCLCPP_WARN(get_logger(), "Socket error on %s, reopening", transport_->describe().c_str());
        transport_->close();
        {
          std::lock_guard<std::mutex> lock(mutex_);
          stats_.link_open = false;
          stats_.link_error = "socket error";
        }
        sleepFor(reconnect_period_);
        break;
      }
    }
  }
}

void CameraTriggerNode::handleLine(const ReceivedLine & line)
{
  if (!line.kernel_timestamp && !logged_no_kernel_timestamp_) {
    logged_no_kernel_timestamp_ = true;
    RCLCPP_WARN(
      get_logger(),
      "No kernel receive timestamps on this socket: using the clock at read time instead (slightly less accurate)");
    std::lock_guard<std::mutex> lock(mutex_);
    stats_.kernel_timestamps = false;
  }

  const ParseResult result = parseLine(line.text, pulses_per_second_);
  if (const auto * trigger = std::get_if<TriggerReport>(&result)) {
    handleTrigger(*trigger, line);
  } else if (const auto * boot = std::get_if<BootReport>(&result)) {
    handleBoot(*boot, line);
  } else {
    countBadLine(std::get<ParseError>(result), line.source);
  }
}

void CameraTriggerNode::handleTrigger(const TriggerReport & report, const ReceivedLine & line)
{
  const SteadyTime now = SteadyClock::now();

  const SequenceResult sequence = sequence_.update(report.seq);
  if (!sequence.accepted()) {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.reports_received;
    if (sequence.verdict == SequenceVerdict::kDuplicate) {
      ++stats_.duplicates;
    } else {
      ++stats_.late;
    }
    return;
  }
  if (sequence.verdict == SequenceVerdict::kReboot) {
    timer_->reset();
    RCLCPP_WARN(
      get_logger(),
      "Pulse sequence jumped back to %u: the trigger board restarted (no BOOT line received)",
      report.seq);
  } else if (sequence.verdict == SequenceVerdict::kGap) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "%u pulse report(s) missing before seq %u", sequence.missing, report.seq);
  }

  const bool had_offset = timer_->offsetEstablished();
  const int64_t old_offset = timer_->offset();
  const PulseTime time = timer_->compute(report, line.t_rx_ns);
  if (timer_->offsetEstablished() && (!had_offset || timer_->offset() != old_offset)) {
    RCLCPP_INFO(get_logger(), "Pulse times locked to the PPS grid (seq %u)", report.seq);
  }
  if (time.locked && !time.residual_ok) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Pulse report arrived %.1f ms after its grid time (expected 0 to %.0f ms): system clock not within %.0f ms of "
      "GPS, or a stale report. Check chrony (#560). Stamps are marked time_valid=false.",
      toMs(time.residual_ns),
      max_latency_s_ * 1e3,
      max_latency_s_ * 1e3);
  } else if (time.locked && !time.offset_ok && had_offset) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Board second counter (pps_count %u) disagrees with the clock: stamp marked time_valid=false",
      report.pps_count);
  }

  camera_trigger_msgs::msg::CameraTrigger msg;
  msg.header.stamp = toRosTime(time.stamp_ns);
  msg.header.frame_id = kFrameId;
  msg.seq = report.seq;
  msg.pps_count = report.pps_count;
  msg.pulse_idx = report.pulse_idx;
  msg.pps_locked = report.pps_locked;
  msg.time_valid = time.time_valid;
  msg.strobe_mask = report.strobe_mask;
  msg.strobe_mask_valid = report.strobe_mask_valid;
  msg.receive_time = toRosTime(line.t_rx_ns);
  trigger_pub_->publish(msg);

  bool started_receiving = false;
  bool lock_changed = false;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats & s = stats_;
    ++s.reports_received;
    ++s.pulses_published;
    if (sequence.verdict == SequenceVerdict::kGap) {
      ++s.gaps;
      s.missing_pulses += sequence.missing;
      s.missing_events.emplace_back(now, sequence.missing);
    } else if (sequence.verdict == SequenceVerdict::kReboot) {
      ++s.board_starts;
    }
    if (!time.time_valid) {
      ++s.invalid_stamps;
    }

    lock_changed = s.have_pulse && s.last_report.pps_locked != report.pps_locked;
    if (!s.receiving) {
      s.receiving = true;
      s.receiving_since = now;
      started_receiving = true;
    }
    s.have_pulse = true;
    s.last_pulse_time = now;
    s.last_report = report;
    s.last_time = time;
    s.last_source = line.source;
    s.offset_established = timer_->offsetEstablished();
    if (time.locked) {
      s.have_residual = true;
      s.last_residual_ns = time.residual_ns;
      if (!time.residual_ok) {
        s.residual_failed = true;
        s.last_residual_failure = now;
        s.last_bad_residual_ns = time.residual_ns;
      }
    }
    s.pulse_times.push_back(now);

    s.strobes_measured = report.strobe_mask_valid;
    if (report.strobe_mask_valid) {
      for (std::size_t bit = 0; bit < kNumStrobes; ++bit) {
        if ((report.strobe_mask >> bit) & 1U) {
          s.strobe_misses[bit] = 0;
        } else if (s.strobe_misses[bit] < UINT32_MAX) {
          ++s.strobe_misses[bit];
        }
      }
    }
    pruneHistory(now);
  }

  if (started_receiving) {
    RCLCPP_INFO(get_logger(), "Receiving pulse reports from %s", line.source.c_str());
  }
  if (lock_changed) {
    if (report.pps_locked) {
      RCLCPP_INFO(get_logger(), "Trigger board locked to PPS again");
    } else {
      RCLCPP_WARN(get_logger(), "Trigger board lost PPS lock: stamps are approximate (time_valid=false)");
    }
  }
}

void CameraTriggerNode::handleBoot(const BootReport & boot, const ReceivedLine & line)
{
  sequence_.reset();
  timer_->reset();
  RCLCPP_INFO(
    get_logger(), "Trigger board started, firmware %s (from %s)", boot.fw_version.c_str(), line.source.c_str());
  std::lock_guard<std::mutex> lock(mutex_);
  stats_.fw_version = boot.fw_version;
  stats_.last_source = line.source;
  ++stats_.board_starts;
}

void CameraTriggerNode::countBadLine(ParseError error, const std::string & source)
{
  if (error == ParseError::kUnknownType) {
    // Line types this node doesn't handle yet (e.g. STAT from newer firmware) are not errors.
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.unknown_lines;
    return;
  }
  {
    const SteadyTime now = SteadyClock::now();
    std::lock_guard<std::mutex> lock(mutex_);
    ++stats_.bad_lines;
    ++stats_.bad_lines_by_type[static_cast<std::size_t>(error)];
    stats_.bad_line_times.push_back(now);
    pruneHistory(now);
  }
  if (error == ParseError::kBadVersion) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(),
      *get_clock(),
      10000,
      "Report from %s uses a protocol version this node doesn't support (it supports version %u): update the node "
      "or the firmware",
      source.c_str(),
      kProtocolVersion);
  } else {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000, "Rejected report from %s: %s", source.c_str(), toString(error));
  }
}

void CameraTriggerNode::pruneHistory(SteadyTime now)
{
  Stats & s = stats_;
  while (!s.pulse_times.empty() && now - s.pulse_times.front() > kRateWindow) {
    s.pulse_times.pop_front();
  }
  while (!s.missing_events.empty() && now - s.missing_events.front().first > kEventWindow) {
    s.missing_events.pop_front();
  }
  while (!s.bad_line_times.empty() && now - s.bad_line_times.front() > kEventWindow) {
    s.bad_line_times.pop_front();
  }
}

void CameraTriggerNode::publishDiagnostics()
{
  const SteadyTime now = SteadyClock::now();
  DiagnosticStatus status;
  status.name = "camera_trigger";
  status.hardware_id = kHardwareId;
  uint8_t level = DiagnosticStatus::OK;
  std::vector<std::string> problems;
  const auto problem = [&level, &problems](uint8_t problem_level, std::string text) {
    level = std::max(level, problem_level);
    problems.push_back(std::move(text));
  };
  const auto value = [&status](std::string key, std::string text) {
    diagnostic_msgs::msg::KeyValue pair;
    pair.key = std::move(key);
    pair.value = std::move(text);
    status.values.push_back(std::move(pair));
  };

  bool stopped_receiving = false;
  double silence_s = 0.0;
  std::string endpoint;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    Stats & s = stats_;
    pruneHistory(now);
    endpoint = s.endpoint;

    const bool stale = !s.have_pulse || now - s.last_pulse_time > stale_timeout_;
    silence_s = s.have_pulse ? std::chrono::duration<double>(now - s.last_pulse_time).count() : 0.0;
    if (stale && s.receiving) {
      s.receiving = false;
      stopped_receiving = true;
    }

    uint64_t missing_recent = 0;
    for (const auto & event : s.missing_events) {
      missing_recent += event.second;
    }
    const double rate_hz =
      static_cast<double>(s.pulse_times.size()) / std::chrono::duration<double>(kRateWindow).count();

    // Messages stay the same while a condition lasts (live numbers go in the key/values),
    // so monitoring tools see one event per problem rather than a new one every update.

    // ERROR: no link, or no pulses
    if (!s.link_open) {
      problem(DiagnosticStatus::ERROR, "cannot listen on " + s.endpoint + ": " + s.link_error);
    } else if (stale) {
      problem(DiagnosticStatus::ERROR, "no packets from STM32 on " + s.endpoint + (s.have_pulse ? "" : " since start"));
    } else {
      // WARN: receiving, but something is off
      if (!s.last_report.pps_locked) {
        problem(DiagnosticStatus::WARN, "PPS not locked: stamps are approximate");
      }
      if (missing_recent > 0) {
        problem(DiagnosticStatus::WARN, std::to_string(missing_recent) + " pulse report(s) missing in the last 10 s");
      }
      if (now - s.receiving_since >= kRateWindow && std::abs(rate_hz - pulses_per_second_) > kRateToleranceHz) {
        problem(
          DiagnosticStatus::WARN,
          "pulse rate is not " + std::to_string(pulses_per_second_) + " +/- " + fixed(kRateToleranceHz, 1) + " Hz");
      }
      if (s.residual_failed && now - s.last_residual_failure < kResidualWindow) {
        problem(
          DiagnosticStatus::WARN,
          "pulse reports arrive outside 0-" + fixed(max_latency_s_ * 1e3, 0) +
            " ms after their pulse: system clock not synced to GPS? Check chrony (#560)");
      }
      if (s.strobes_measured) {
        for (std::size_t bit = 0; bit < kNumStrobes; ++bit) {
          if (((expected_strobe_mask_ >> bit) & 1U) && s.strobe_misses[bit] >= strobe_miss_warn_count_) {
            problem(
              DiagnosticStatus::WARN,
              "no strobe from " + strobeName(bit) + " for " + std::to_string(strobe_miss_warn_count_) +
                "+ pulses in a row");
          }
        }
      }
    }
    if (!s.bad_line_times.empty()) {
      problem(DiagnosticStatus::WARN, std::to_string(s.bad_line_times.size()) + " bad report line(s) in the last 10 s");
    }

    // Key/values
    value("endpoint", s.endpoint);
    value("sender", s.last_source.empty() ? "none yet" : s.last_source);
    value("firmware_version", s.fw_version);
    value("rate_hz", fixed(rate_hz, 1));
    value("seconds_since_last_pulse", s.have_pulse ? fixed(silence_s, 1) : "none yet");
    value("pulses_received", std::to_string(s.reports_received));
    value("pulses_published", std::to_string(s.pulses_published));
    value("last_seq", s.have_pulse ? std::to_string(s.last_report.seq) : "none");
    value("last_pps_count", s.have_pulse ? std::to_string(s.last_report.pps_count) : "none");
    value("last_pulse_idx", s.have_pulse ? std::to_string(s.last_report.pulse_idx) : "none");
    value("pps_locked", s.have_pulse ? (s.last_report.pps_locked ? "true" : "false") : "unknown");
    value("time_valid", s.have_pulse ? (s.last_time.time_valid ? "true" : "false") : "unknown");
    value("second_offset_established", s.offset_established ? "true" : "false");
    value("last_residual_ms", s.have_residual ? fixed(toMs(s.last_residual_ns), 2) : "none");
    value("last_bad_residual_ms", s.residual_failed ? fixed(toMs(s.last_bad_residual_ns), 2) : "none");
    value("gaps_total", std::to_string(s.gaps));
    value("missing_pulses_total", std::to_string(s.missing_pulses));
    value("missing_pulses_last_10s", std::to_string(missing_recent));
    value("duplicates_total", std::to_string(s.duplicates));
    value("late_total", std::to_string(s.late));
    value("board_starts", std::to_string(s.board_starts));
    value("invalid_stamps_total", std::to_string(s.invalid_stamps));

    std::vector<std::string> bad_by_type;
    for (std::size_t i = 0; i < kNumParseErrors; ++i) {
      if (s.bad_lines_by_type[i] > 0) {
        bad_by_type.push_back(
          std::string(toString(static_cast<ParseError>(i))) + "=" + std::to_string(s.bad_lines_by_type[i]));
      }
    }
    value("bad_lines_total", std::to_string(s.bad_lines));
    value("bad_lines_by_type", bad_by_type.empty() ? "none" : joined(bad_by_type, ", "));
    value("unknown_lines_total", std::to_string(s.unknown_lines));
    value("foreign_packets_total", std::to_string(s.foreign_packets));
    value("kernel_timestamps", s.kernel_timestamps ? "true" : "false");

    if (!s.have_pulse) {
      value("strobes", "unknown");
    } else if (!s.strobes_measured) {
      value("strobes", "not measured by the board");
    } else {
      value("strobes", hex3(s.last_report.strobe_mask));
    }
    std::vector<std::string> misses;
    for (std::size_t bit = 0; bit < kNumStrobes; ++bit) {
      if (s.strobe_misses[bit] > 0) {
        misses.push_back(strobeName(bit) + "=" + std::to_string(s.strobe_misses[bit]));
      }
    }
    value("strobe_misses_in_a_row", misses.empty() ? "none" : joined(misses, ", "));
  }

  if (stopped_receiving) {
    RCLCPP_WARN(get_logger(), "No pulse reports for %.1f s on %s", silence_s, endpoint.c_str());
  }

  status.level = level;
  status.message = problems.empty() ? "OK" : joined(problems, "; ");
  diagnostic_msgs::msg::DiagnosticArray array;
  array.header.stamp = this->now();
  array.status.push_back(std::move(status));
  diagnostics_pub_->publish(array);
}

}  // namespace camera_trigger

RCLCPP_COMPONENTS_REGISTER_NODE(camera_trigger::CameraTriggerNode)
