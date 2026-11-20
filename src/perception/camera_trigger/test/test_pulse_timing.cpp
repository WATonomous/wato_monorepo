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

#include <cstdint>
#include <random>
#include <stdexcept>

#include <catch2/catch_test_macros.hpp>

#include "camera_trigger/protocol.hpp"
#include "camera_trigger/pulse_timing.hpp"

using camera_trigger::floorDiv;
using camera_trigger::kNanosPerSecond;
using camera_trigger::PulseTime;
using camera_trigger::PulseTimer;
using camera_trigger::PulseTimingConfig;
using camera_trigger::TriggerReport;

namespace
{

constexpr int64_t kMs = 1000000;
constexpr int64_t kPeriodNs = 50 * kMs;  // 20 Hz
constexpr int64_t kBaseSecond = 1763380000;  // a whole UTC second (Nov 2025)
constexpr uint32_t kPpsCountAtBase = 7;  // the board's second counter at kBaseSecond
constexpr uint32_t kAdoptCount = 20;

struct Pulse
{
  TriggerReport report;
  int64_t true_ns;  ///< when the pulse really fired (GPS time)
};

// The k-th locked pulse after kBaseSecond, as the board reports it.
Pulse pulse(uint32_t k, int64_t pps_count_shift = 0)
{
  Pulse p{};
  p.report.seq = 1000 + k;
  p.report.pps_count = static_cast<uint32_t>(kPpsCountAtBase + k / 20 + pps_count_shift);
  p.report.pulse_idx = static_cast<uint8_t>(k % 20);
  p.report.pps_locked = true;
  p.report.strobe_mask = 0xFFF;
  p.report.strobe_mask_valid = true;
  p.true_ns = (kBaseSecond + k / 20) * kNanosPerSecond + (k % 20) * kPeriodNs;
  return p;
}

PulseTimer makeTimer()
{
  return PulseTimer(PulseTimingConfig{});  // 20 Hz, 10 ms expected, 100 ms max, adopt after 20
}

// Receive time of a pulse: true time + network/processing latency + error of the computer clock.
int64_t receiveTime(const Pulse & p, int64_t latency_ns, int64_t clock_error_ns = 0)
{
  return p.true_ns + latency_ns + clock_error_ns;
}

// Feeds pulses [first, last) so the second offset gets established.
void warmUp(PulseTimer & timer, uint32_t first = 0, uint32_t last = kAdoptCount)
{
  for (uint32_t k = first; k < last; ++k) {
    const Pulse p = pulse(k);
    timer.compute(p.report, receiveTime(p, 10 * kMs));
  }
  REQUIRE(timer.offsetEstablished());
}

}  // namespace

TEST_CASE("floorDiv rounds toward negative infinity", "[timing]")
{
  CHECK(floorDiv(9, 10) == 0);
  CHECK(floorDiv(10, 10) == 1);
  CHECK(floorDiv(-1, 10) == -1);
  CHECK(floorDiv(-10, 10) == -1);
  CHECK(floorDiv(-11, 10) == -2);
}

TEST_CASE("Second offset is established after 20 consistent pulses", "[timing]")
{
  PulseTimer timer = makeTimer();
  for (uint32_t k = 0; k < kAdoptCount - 1; ++k) {
    const Pulse p = pulse(k);
    const PulseTime t = timer.compute(p.report, receiveTime(p, 10 * kMs));
    CHECK(t.stamp_ns == p.true_ns);  // the stamp is already right...
    CHECK(t.residual_ok);
    CHECK_FALSE(t.offset_ok);  // ...but not yet trusted
    CHECK_FALSE(t.time_valid);
  }
  const Pulse p = pulse(kAdoptCount - 1);
  const PulseTime t = timer.compute(p.report, receiveTime(p, 10 * kMs));
  CHECK(t.time_valid);
  CHECK(timer.offsetEstablished());
  CHECK(timer.offset() == kBaseSecond - kPpsCountAtBase);
}

TEST_CASE("Every pulse index snaps to its grid point", "[timing]")
{
  PulseTimer timer = makeTimer();
  warmUp(timer);
  for (uint32_t k = kAdoptCount; k < kAdoptCount + 40; ++k) {
    const Pulse p = pulse(k);
    const PulseTime t = timer.compute(p.report, receiveTime(p, 10 * kMs));
    INFO("pulse_idx " << static_cast<int>(p.report.pulse_idx));
    CHECK(t.stamp_ns == p.true_ns);
    CHECK(t.stamp_ns % kPeriodNs == 0);
    CHECK(t.residual_ns == 10 * kMs);
    CHECK(t.locked);
    CHECK(t.time_valid);
  }
}

TEST_CASE("Latency jitter does not move the stamp", "[timing]")
{
  PulseTimer timer = makeTimer();
  warmUp(timer);
  std::mt19937 rng(42);
  std::uniform_int_distribution<int64_t> latency(kMs / 2, 90 * kMs);
  for (uint32_t k = kAdoptCount; k < kAdoptCount + 400; ++k) {
    const Pulse p = pulse(k);
    const int64_t delay = latency(rng);
    const PulseTime t = timer.compute(p.report, receiveTime(p, delay));
    INFO("pulse " << k << ", latency " << delay << " ns");
    CHECK(t.stamp_ns == p.true_ns);
    CHECK(t.residual_ns == delay);
    CHECK(t.time_valid);
  }
}

TEST_CASE("Second boundaries", "[timing]")
{
  PulseTimer timer = makeTimer();
  warmUp(timer);

  SECTION("last pulse of a second, received after the next second started")
  {
    const Pulse p = pulse(kAdoptCount + 19);
    const PulseTime t = timer.compute(p.report, receiveTime(p, 60 * kMs));
    CHECK(t.stamp_ns == p.true_ns);
    CHECK(t.time_valid);
  }
  SECTION("first pulse of a second, received almost immediately")
  {
    const Pulse p = pulse(kAdoptCount + 20);
    const PulseTime t = timer.compute(p.report, receiveTime(p, kMs / 2));
    CHECK(t.stamp_ns == p.true_ns);
    CHECK(t.time_valid);
  }
  SECTION("a very late report still snaps to the right pulse, but is not trusted")
  {
    const Pulse p = pulse(kAdoptCount + 5);
    const PulseTime t = timer.compute(p.report, receiveTime(p, 400 * kMs));
    CHECK(t.stamp_ns == p.true_ns);
    CHECK_FALSE(t.residual_ok);
    CHECK_FALSE(t.time_valid);
  }
}

TEST_CASE("Small clock errors", "[timing]")
{
  PulseTimer timer = makeTimer();
  warmUp(timer);
  const Pulse p = pulse(kAdoptCount + 3);

  SECTION("clock 40 ms ahead: residual 45 ms is still plausible, and the stamp is the true pulse time")
  {
    const PulseTime t = timer.compute(p.report, receiveTime(p, 5 * kMs, 40 * kMs));
    CHECK(t.stamp_ns == p.true_ns);
    CHECK(t.residual_ns == 45 * kMs);
    CHECK(t.time_valid);
  }
  SECTION("clock 40 ms behind: negative residual, marked invalid")
  {
    const PulseTime t = timer.compute(p.report, receiveTime(p, 5 * kMs, -40 * kMs));
    CHECK(t.stamp_ns == p.true_ns);
    CHECK(t.residual_ns == -35 * kMs);
    CHECK_FALSE(t.residual_ok);
    CHECK_FALSE(t.time_valid);
  }
  SECTION("clock 0.2 s ahead: residual above max_latency, marked invalid")
  {
    const PulseTime t = timer.compute(p.report, receiveTime(p, 5 * kMs, 200 * kMs));
    CHECK_FALSE(t.residual_ok);
    CHECK_FALSE(t.time_valid);
  }
  SECTION("clock 0.2 s behind: marked invalid")
  {
    const PulseTime t = timer.compute(p.report, receiveTime(p, 5 * kMs, -200 * kMs));
    CHECK_FALSE(t.residual_ok);
    CHECK_FALSE(t.time_valid);
  }
}

TEST_CASE("Whole-second clock errors cannot be detected (documented limitation)", "[timing]")
{
  // The board only counts seconds since its boot, so a clock that is off by exactly N seconds
  // looks consistent. This is why time_valid depends on chrony keeping the clock on GPS time.
  for (const int64_t error_s : {int64_t{2}, int64_t{-2}}) {
    PulseTimer timer = makeTimer();
    for (uint32_t k = 0; k < kAdoptCount + 5; ++k) {
      const Pulse p = pulse(k);
      const PulseTime t = timer.compute(p.report, receiveTime(p, 10 * kMs, error_s * kNanosPerSecond));
      CHECK(t.stamp_ns == p.true_ns + error_s * kNanosPerSecond);
      CHECK(t.stamp_ns % kPeriodNs == 0);
      if (k >= kAdoptCount - 1) {
        CHECK(t.time_valid);
      }
    }
  }
}

TEST_CASE("A single pulse with an inconsistent second counter is marked invalid", "[timing]")
{
  PulseTimer timer = makeTimer();
  warmUp(timer);
  const Pulse odd = pulse(kAdoptCount, 1);  // pps_count one too high
  const PulseTime bad = timer.compute(odd.report, receiveTime(odd, 10 * kMs));
  CHECK_FALSE(bad.offset_ok);
  CHECK_FALSE(bad.time_valid);

  const Pulse normal = pulse(kAdoptCount + 1);
  CHECK(timer.compute(normal.report, receiveTime(normal, 10 * kMs)).time_valid);
  CHECK(timer.offset() == kBaseSecond - kPpsCountAtBase);
}

TEST_CASE("A new second offset is adopted after 20 agreeing pulses", "[timing]")
{
  PulseTimer timer = makeTimer();
  warmUp(timer);
  const int64_t shift = 5;
  for (uint32_t i = 0; i < kAdoptCount; ++i) {
    const Pulse p = pulse(kAdoptCount + i, shift);
    const PulseTime t = timer.compute(p.report, receiveTime(p, 10 * kMs));
    CHECK(t.stamp_ns == p.true_ns);
    if (i < kAdoptCount - 1) {
      CHECK_FALSE(t.time_valid);
    } else {
      CHECK(t.time_valid);
    }
  }
  CHECK(timer.offset() == kBaseSecond - kPpsCountAtBase - shift);
}

TEST_CASE("Unlocked pulses", "[timing]")
{
  PulseTimer timer = makeTimer();
  warmUp(timer);

  Pulse p = pulse(kAdoptCount);
  p.report.pps_locked = false;
  const int64_t t_rx = receiveTime(p, 23 * kMs);
  const PulseTime t = timer.compute(p.report, t_rx);
  CHECK_FALSE(t.locked);
  CHECK_FALSE(t.time_valid);
  CHECK(t.stamp_ns == t_rx - 10 * kMs);  // receive time minus expected latency

  SECTION("relocking with the same second offset is valid again immediately")
  {
    const Pulse relocked = pulse(kAdoptCount + 1);
    CHECK(timer.compute(relocked.report, receiveTime(relocked, 10 * kMs)).time_valid);
  }
}

TEST_CASE("An unlocked pulse breaks an offset streak in progress", "[timing]")
{
  PulseTimer timer = makeTimer();
  for (uint32_t k = 0; k < 10; ++k) {
    const Pulse p = pulse(k);
    timer.compute(p.report, receiveTime(p, 10 * kMs));
  }
  Pulse unlocked = pulse(10);
  unlocked.report.pps_locked = false;
  timer.compute(unlocked.report, receiveTime(unlocked, 10 * kMs));

  for (uint32_t i = 0; i < kAdoptCount - 1; ++i) {
    const Pulse p = pulse(11 + i);
    CHECK_FALSE(timer.compute(p.report, receiveTime(p, 10 * kMs)).time_valid);
  }
  const Pulse p = pulse(11 + kAdoptCount - 1);
  CHECK(timer.compute(p.report, receiveTime(p, 10 * kMs)).time_valid);
}

TEST_CASE("Reset forgets the second offset", "[timing]")
{
  PulseTimer timer = makeTimer();
  warmUp(timer);
  timer.reset();
  CHECK_FALSE(timer.offsetEstablished());
  const Pulse p = pulse(kAdoptCount);
  CHECK_FALSE(timer.compute(p.report, receiveTime(p, 10 * kMs)).time_valid);
}

TEST_CASE("Rates that don't divide one second", "[timing]")
{
  PulseTimingConfig config;
  config.pulses_per_second = 30;
  const PulseTimer timer(config);
  CHECK(timer.pulseOffsetNs(0) == 0);
  CHECK(timer.pulseOffsetNs(1) == 33333333);
  CHECK(timer.pulseOffsetNs(29) == 966666667);
}

TEST_CASE("Unusable configurations are rejected", "[timing]")
{
  PulseTimingConfig config;
  config.pulses_per_second = 0;
  CHECK_THROWS_AS(PulseTimer(config), std::invalid_argument);
  config.pulses_per_second = 300;
  CHECK_THROWS_AS(PulseTimer(config), std::invalid_argument);

  config = PulseTimingConfig{};
  config.expected_latency_ns = -1;
  CHECK_THROWS_AS(PulseTimer(config), std::invalid_argument);

  config = PulseTimingConfig{};
  config.max_latency_ns = 0;
  CHECK_THROWS_AS(PulseTimer(config), std::invalid_argument);

  config = PulseTimingConfig{};
  config.offset_adopt_count = 0;
  CHECK_THROWS_AS(PulseTimer(config), std::invalid_argument);
}
