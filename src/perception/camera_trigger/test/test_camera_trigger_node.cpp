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

// Runs the real node against a fake board (a UDP socket on loopback) and checks what it publishes.

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <chrono>
#include <cstdint>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <builtin_interfaces/msg/time.hpp>
#include <camera_trigger_msgs/msg/camera_trigger.hpp>
#include <catch2/catch_test_macros.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <rclcpp/rclcpp.hpp>
#include <wato_test/wato_test.hpp>

#include "camera_trigger/camera_trigger_node.hpp"
#include "camera_trigger/protocol.hpp"
#include "camera_trigger/pulse_timing.hpp"
#include "camera_trigger/transport.hpp"

using camera_trigger::CameraTriggerNode;
using camera_trigger::frameLine;
using camera_trigger::kNanosPerSecond;
using camera_trigger::systemTimeNs;
using camera_trigger_msgs::msg::CameraTrigger;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace
{

constexpr uint32_t kRate = 20;
constexpr int64_t kPeriodNs = kNanosPerSecond / kRate;
constexpr int64_t kReportDelayNs = 2000000;  // 2 ms from pulse to report
constexpr auto kTimeout = std::chrono::seconds(10);

/// Stands in for the STM32.
class FakeBoard
{
public:
  explicit FakeBoard(uint16_t port)
  {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(fd_ >= 0);
    dest_.sin_family = AF_INET;
    dest_.sin_port = htons(port);
    REQUIRE(inet_pton(AF_INET, "127.0.0.1", &dest_.sin_addr) == 1);
  }

  ~FakeBoard()
  {
    ::close(fd_);
  }

  FakeBoard(const FakeBoard &) = delete;
  FakeBoard & operator=(const FakeBoard &) = delete;

  void sendRaw(const std::string & line)
  {
    const ssize_t sent =
      ::sendto(fd_, line.data(), line.size(), 0, reinterpret_cast<const sockaddr *>(&dest_), sizeof(dest_));
    REQUIRE(sent == static_cast<ssize_t>(line.size()));
  }

  void sendTrigger(uint32_t seq, uint32_t pps_count, uint32_t pulse_idx, bool locked = true)
  {
    sendRaw(frameLine(
      "TRG,1," + std::to_string(seq) + "," + std::to_string(pps_count) + "," + std::to_string(pulse_idx) + "," +
      (locked ? "1" : "0") + ",FFF"));
  }

private:
  int fd_{-1};
  sockaddr_in dest_{};
};

/// Asks the OS for an unused UDP port.
uint16_t freeUdpPort()
{
  const int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
  REQUIRE(fd >= 0);
  sockaddr_in addr{};
  addr.sin_family = AF_INET;
  REQUIRE(inet_pton(AF_INET, "127.0.0.1", &addr.sin_addr) == 1);
  REQUIRE(::bind(fd, reinterpret_cast<const sockaddr *>(&addr), sizeof(addr)) == 0);
  socklen_t len = sizeof(addr);
  REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr *>(&addr), &len) == 0);
  ::close(fd);
  return ntohs(addr.sin_port);
}

rclcpp::NodeOptions nodeOptions(uint16_t port, const std::string & bind_address = "127.0.0.1")
{
  return rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("udp_bind_address", bind_address), rclcpp::Parameter("udp_port", static_cast<int64_t>(port))});
}

int64_t stampNs(const builtin_interfaces::msg::Time & time)
{
  return static_cast<int64_t>(time.sec) * kNanosPerSecond + time.nanosec;
}

void sleepUntilSystemTime(int64_t t_ns)
{
  const int64_t delay = t_ns - systemTimeNs();
  if (delay > 0) {
    std::this_thread::sleep_for(std::chrono::nanoseconds(delay));
  }
}

/// The node's entry in a /diagnostics message, or nullptr if it has none.
const DiagnosticStatus * triggerStatus(const DiagnosticArray & array)
{
  for (const auto & status : array.status) {
    if (status.name == "camera_trigger") {
      return &status;
    }
  }
  return nullptr;
}

std::string valueOf(const DiagnosticStatus & status, const std::string & key)
{
  for (const auto & pair : status.values) {
    if (pair.key == key) {
      return pair.value;
    }
  }
  return "<missing>";
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

template <typename T>
bool ready(std::future<T> & future)
{
  return future.wait_for(std::chrono::seconds(0)) == std::future_status::ready;
}

}  // namespace

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "camera_trigger_node end to end", "[node]")
{
  const uint16_t port = freeUdpPort();
  auto node = std::make_shared<CameraTriggerNode>(nodeOptions(port));
  auto triggers = std::make_shared<wato::test::SubscriberTestNode<CameraTrigger>>(
    "/camera_trigger", "trigger_listener", rclcpp::QoS(rclcpp::KeepLast(100)).reliable());
  auto diagnostics = std::make_shared<wato::test::SubscriberTestNode<DiagnosticArray>>(
    "/diagnostics", "diagnostics_listener", rclcpp::QoS(10));

  // Wait for the "nothing received" error first: it also means the socket is open.
  auto silent_at_start = diagnostics->expect_message([](const DiagnosticArray & array) {
    const auto * status = triggerStatus(array);
    return status != nullptr && status->level == DiagnosticStatus::ERROR &&
           contains(status->message, "no packets from STM32") && contains(status->message, "since start");
  });

  add_node(node);
  add_node(triggers);
  add_node(diagnostics);
  start_spinning();
  REQUIRE(triggers->wait_for_publishers(1));
  REQUIRE(diagnostics->wait_for_publishers(1));
  REQUIRE(silent_at_start.wait_for(kTimeout) == std::future_status::ready);

  FakeBoard board(port);

  SECTION("pulses sent on the PPS grid get exact stamps, and silence afterwards is an error")
  {
    auto valid_pulse = triggers->expect_message([](const CameraTrigger & msg) { return msg.time_valid; });
    auto healthy = diagnostics->expect_message([](const DiagnosticArray & array) {
      const auto * status = triggerStatus(array);
      return status != nullptr && status->level == DiagnosticStatus::OK && valueOf(*status, "time_valid") == "true";
    });

    // Pulses sit on the 50 ms grid and are reported 2 ms late, like the real board. Keep sending
    // until the node is healthy, since a slow CI machine can delay the first few reports.
    const int64_t first_pulse_ns = (systemTimeNs() / kPeriodNs + 2) * kPeriodNs;
    const int64_t first_second = first_pulse_ns / kNanosPerSecond;
    const auto deadline = std::chrono::steady_clock::now() + kTimeout;
    uint32_t seq = 0;
    while (!(ready(valid_pulse) && ready(healthy)) && std::chrono::steady_clock::now() < deadline) {
      const int64_t pulse_ns = first_pulse_ns + static_cast<int64_t>(seq) * kPeriodNs;
      sleepUntilSystemTime(pulse_ns + kReportDelayNs);
      const auto pps_count = static_cast<uint32_t>(pulse_ns / kNanosPerSecond - first_second);
      const auto pulse_idx = static_cast<uint32_t>((pulse_ns % kNanosPerSecond) / kPeriodNs);
      board.sendTrigger(seq, pps_count, pulse_idx);
      ++seq;
    }
    REQUIRE(ready(valid_pulse));
    REQUIRE(ready(healthy));

    const CameraTrigger msg = valid_pulse.get();
    CHECK(msg.header.frame_id == "camera_trigger");
    CHECK(msg.pps_locked);
    CHECK(msg.strobe_mask_valid);
    CHECK(msg.strobe_mask == 0x0FFF);
    // Stamped with the pulse time, not the receive time.
    CHECK(stampNs(msg.header.stamp) == first_pulse_ns + static_cast<int64_t>(msg.seq) * kPeriodNs);
    CHECK(stampNs(msg.receive_time) > stampNs(msg.header.stamp));
    // The first 19 pulses are never marked valid.
    CHECK(msg.seq >= 19);

    // Stop sending, as if the cable was pulled.
    auto silent = diagnostics->expect_message([](const DiagnosticArray & array) {
      const auto * status = triggerStatus(array);
      return status != nullptr && status->level == DiagnosticStatus::ERROR &&
             contains(status->message, "no packets from STM32") && !contains(status->message, "since start");
    });
    REQUIRE(silent.wait_for(kTimeout) == std::future_status::ready);
  }

  SECTION("lost, duplicated and corrupted reports are counted and never published")
  {
    auto after_gap = triggers->expect_message([](const CameraTrigger & msg) { return msg.seq == 5; });

    board.sendRaw(frameLine("BOOT,1,test-fw"));
    board.sendTrigger(0, 0, 0);
    board.sendTrigger(1, 0, 1);
    board.sendTrigger(1, 0, 1);  // duplicate
    board.sendRaw("TRG,1,2,0,2,1,FFF*00\r\n");  // wrong checksum
    board.sendRaw(frameLine("STAT,1,12,RUN"));  // unknown line type
    board.sendTrigger(5, 0, 5);  // skips 2, 3 and 4

    REQUIRE(after_gap.wait_for(kTimeout) == std::future_status::ready);
    // Only seq 0, 1 and 5 get through.
    CHECK(triggers->get_message_count() == 3);

    auto counted = diagnostics->expect_message([](const DiagnosticArray & array) {
      const auto * status = triggerStatus(array);
      return status != nullptr && valueOf(*status, "last_seq") == "5";
    });
    REQUIRE(counted.wait_for(kTimeout) == std::future_status::ready);
    const DiagnosticArray array = counted.get();
    const DiagnosticStatus & status = *triggerStatus(array);
    CHECK(status.level != DiagnosticStatus::OK);
    CHECK(valueOf(status, "firmware_version") == "test-fw");
    CHECK(valueOf(status, "pulses_received") == "4");
    CHECK(valueOf(status, "pulses_published") == "3");
    CHECK(valueOf(status, "gaps_total") == "1");
    CHECK(valueOf(status, "missing_pulses_total") == "3");
    CHECK(valueOf(status, "duplicates_total") == "1");
    CHECK(valueOf(status, "bad_lines_total") == "1");
    CHECK(valueOf(status, "bad_lines_by_type") == "bad_checksum=1");
    CHECK(valueOf(status, "unknown_lines_total") == "1");
    CHECK(valueOf(status, "board_starts") == "1");
  }

  SECTION("a pulse not locked to PPS is published with time_valid false and a warning")
  {
    auto unlocked = triggers->expect_message([](const CameraTrigger & msg) { return msg.seq == 7; });
    auto warned = diagnostics->expect_message([](const DiagnosticArray & array) {
      const auto * status = triggerStatus(array);
      return status != nullptr && contains(status->message, "PPS not locked");
    });

    // Keep sending, or the silence error would hide the PPS warning.
    const auto deadline = std::chrono::steady_clock::now() + kTimeout;
    uint32_t seq = 7;
    while (!ready(warned) && std::chrono::steady_clock::now() < deadline) {
      board.sendTrigger(seq, seq / kRate, seq % kRate, false);
      ++seq;
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
    REQUIRE(ready(unlocked));
    REQUIRE(ready(warned));

    const CameraTrigger msg = unlocked.get();
    CHECK_FALSE(msg.pps_locked);
    CHECK_FALSE(msg.time_valid);
    const DiagnosticArray array = warned.get();
    CHECK(triggerStatus(array)->level == DiagnosticStatus::WARN);
  }
}

TEST_CASE_METHOD(wato::test::TestExecutorFixture, "camera_trigger_node rejects bad parameters", "[node]")
{
  const uint16_t port = freeUdpPort();
  CHECK_THROWS_AS(std::make_shared<CameraTriggerNode>(nodeOptions(port, "not-an-address")), std::invalid_argument);
  CHECK_THROWS_AS(
    std::make_shared<CameraTriggerNode>(nodeOptions(port).append_parameter_override("udp_source_ip", "10.8.0.999")),
    std::invalid_argument);
  CHECK_THROWS_AS(
    std::make_shared<CameraTriggerNode>(nodeOptions(port).append_parameter_override("pulses_per_second", 0)),
    std::invalid_argument);
  CHECK_THROWS_AS(
    std::make_shared<CameraTriggerNode>(
      nodeOptions(port).append_parameter_override("expected_strobe_bits", std::vector<int64_t>{0, 12})),
    std::invalid_argument);
}
