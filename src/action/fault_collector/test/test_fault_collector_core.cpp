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

#include <string>

#include <catch2/catch_test_macros.hpp>

#include "fault_collector/fault_collector_core.hpp"

namespace wato
{

namespace
{

fault_collector::FaultRecord make_record(
  uint8_t level,
  const std::string & source,
  const std::string & message,
  const std::string & id = "",
  bool active = true)
{
  fault_collector::FaultRecord record;
  record.level = level;
  record.source = source;
  record.message = message;
  record.id = id;
  record.active = active;
  return record;
}

}  // namespace

// ---------------------------------------------------------------------------
// Tier basics
// ---------------------------------------------------------------------------

TEST_CASE("A single MILD fault trips neither tier; a CRITICAL fault trips both", "[fault_collector_core][tiers]")
{
  fault_collector::FaultCollectorConfig cfg;  // disarm_level=1, brake_level=2
  fault_collector::FaultCollectorCore core(cfg);

  auto result = core.report(make_record(fault_collector::kMild, "camera", "low framerate"), 0.0);
  REQUIRE(result == fault_collector::ReportResult::kAccepted);
  REQUIRE_FALSE(core.should_brake());
  REQUIRE_FALSE(core.should_disarm());

  result = core.report(make_record(fault_collector::kCritical, "lidar", "sensor offline"), 0.1);
  REQUIRE(result == fault_collector::ReportResult::kAccepted);
  REQUIRE(core.should_brake());
  REQUIRE(core.should_disarm());
}

// ---------------------------------------------------------------------------
// Keying behaviour
// ---------------------------------------------------------------------------

TEST_CASE("Same (source, id) reported twice results in one entry with count == 2", "[fault_collector_core][keying]")
{
  fault_collector::FaultCollectorConfig cfg;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kModerate, "gps", "low satellite count", "low_sat"), 0.0);
  core.report(make_record(fault_collector::kModerate, "gps", "low satellite count", "low_sat"), 1.0);

  REQUIRE(core.active_faults().size() == 1);
  const auto & fault = core.active_faults().begin()->second;
  REQUIRE(fault.count == 2);
  REQUIRE(fault.first_seen_s == 0.0);
  REQUIRE(fault.last_seen_s == 1.0);
}

TEST_CASE("Different sources with the same message produce two separate entries", "[fault_collector_core][keying]")
{
  fault_collector::FaultCollectorConfig cfg;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kSevere, "camera_front", "stale frame"), 0.0);
  core.report(make_record(fault_collector::kSevere, "camera_rear", "stale frame"), 0.0);

  REQUIRE(core.active_faults().size() == 2);
}

// ---------------------------------------------------------------------------
// Expiry
// ---------------------------------------------------------------------------

TEST_CASE(
  "A fault expires after fault_timeout_s and both tiers clear when latch == false", "[fault_collector_core][expiry]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.fault_timeout_s = 5.0;
  cfg.latch = false;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 0.0);
  REQUIRE(core.should_disarm());
  REQUIRE(core.should_brake());

  core.expire(4.0);
  REQUIRE(core.active_faults().size() == 1);
  REQUIRE(core.should_disarm());
  REQUIRE(core.should_brake());

  core.expire(5.1);
  REQUIRE(core.active_faults().empty());
  REQUIRE_FALSE(core.should_disarm());
  REQUIRE_FALSE(core.should_brake());
}

TEST_CASE(
  "With latch == true, both tiers stay tripped after expiry and clear_latch() releases them",
  "[fault_collector_core][expiry][latch]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.fault_timeout_s = 5.0;
  cfg.latch = true;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 0.0);
  REQUIRE(core.should_disarm());
  REQUIRE(core.should_brake());

  core.expire(5.1);
  REQUIRE(core.active_faults().empty());
  REQUIRE(core.should_disarm());  // still latched even though the fault is gone
  REQUIRE(core.should_brake());

  core.clear_latch();
  REQUIRE_FALSE(core.should_disarm());
  REQUIRE_FALSE(core.should_brake());
}

// ---------------------------------------------------------------------------
// Explicit clearing
// ---------------------------------------------------------------------------

TEST_CASE("A report with active == false erases the fault immediately", "[fault_collector_core][clear]")
{
  fault_collector::FaultCollectorConfig cfg;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kModerate, "steering", "calibration drift", "calib_drift"), 0.0);
  REQUIRE(core.active_faults().size() == 1);

  const auto result =
    core.report(make_record(fault_collector::kModerate, "steering", "calibration drift", "calib_drift", false), 1.0);

  REQUIRE(result == fault_collector::ReportResult::kCleared);
  REQUIRE(core.active_faults().empty());
}

// ---------------------------------------------------------------------------
// Validation
// ---------------------------------------------------------------------------

TEST_CASE(
  "Invalid level (0 or 5) and empty source are rejected and do not appear in the table",
  "[fault_collector_core][validation]")
{
  fault_collector::FaultCollectorConfig cfg;
  fault_collector::FaultCollectorCore core(cfg);

  auto result_zero = core.report(make_record(0, "imu", "bad level"), 0.0);
  REQUIRE(result_zero == fault_collector::ReportResult::kRejectedInvalidLevel);

  auto result_five = core.report(make_record(5, "imu", "bad level"), 0.0);
  REQUIRE(result_five == fault_collector::ReportResult::kRejectedInvalidLevel);

  auto result_empty_source = core.report(make_record(fault_collector::kCritical, "", "no source"), 0.0);
  REQUIRE(result_empty_source == fault_collector::ReportResult::kRejectedEmptySource);

  REQUIRE(core.active_faults().empty());
}

TEST_CASE("FaultCollectorConfig::is_valid() rejects disarm_level > brake_level", "[fault_collector_core][validation]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.disarm_level = fault_collector::kModerate;
  cfg.brake_level = fault_collector::kSevere;
  REQUIRE_FALSE(cfg.is_valid());

  cfg.disarm_level = fault_collector::kCritical;
  cfg.brake_level = fault_collector::kSevere;
  REQUIRE(cfg.is_valid());

  cfg.disarm_level = fault_collector::kSevere;
  cfg.brake_level = fault_collector::kSevere;
  REQUIRE(cfg.is_valid());  // equal is allowed
}

// ---------------------------------------------------------------------------
// Worst level
// ---------------------------------------------------------------------------

TEST_CASE("worst_level() returns the numerically lowest active level", "[fault_collector_core][worst_level]")
{
  fault_collector::FaultCollectorConfig cfg;
  fault_collector::FaultCollectorCore core(cfg);

  REQUIRE_FALSE(core.worst_level().has_value());

  core.report(make_record(fault_collector::kMild, "a", "mild issue"), 0.0);
  core.report(make_record(fault_collector::kModerate, "b", "moderate issue"), 0.0);
  core.report(make_record(fault_collector::kSevere, "c", "severe issue"), 0.0);

  REQUIRE(core.worst_level().has_value());
  REQUIRE(*core.worst_level() == fault_collector::kSevere);

  core.report(make_record(fault_collector::kCritical, "d", "critical issue"), 0.0);
  REQUIRE(*core.worst_level() == fault_collector::kCritical);
}

// ---------------------------------------------------------------------------
// Configurable tier thresholds
// ---------------------------------------------------------------------------

TEST_CASE(
  "brake_level == 2 makes a SEVERE fault brake while MODERATE does not", "[fault_collector_core][brake][threshold]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.brake_level = fault_collector::kSevere;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kModerate, "suspension", "sensor noisy"), 0.0);
  REQUIRE_FALSE(core.should_brake());

  core.report(make_record(fault_collector::kSevere, "brakes", "pressure low"), 0.0);
  REQUIRE(core.should_brake());
}

TEST_CASE(
  "disarm_level=1, brake_level=2: a SEVERE fault sets should_brake() but NOT should_disarm()",
  "[fault_collector_core][tiers][threshold]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.disarm_level = fault_collector::kCritical;
  cfg.brake_level = fault_collector::kSevere;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kSevere, "brakes", "pressure low"), 0.0);

  REQUIRE(core.should_brake());
  REQUIRE_FALSE(core.should_disarm());
}

TEST_CASE("A CRITICAL fault sets both should_disarm() and should_brake()", "[fault_collector_core][tiers]")
{
  fault_collector::FaultCollectorConfig cfg;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kCritical, "steering", "actuator offline"), 0.0);

  REQUIRE(core.should_disarm());
  REQUIRE(core.should_brake());
}

// ---------------------------------------------------------------------------
// Disarm edge detection
// ---------------------------------------------------------------------------

TEST_CASE(
  "consume_disarm_edge() returns true exactly once for a sustained critical fault",
  "[fault_collector_core][disarm_edge]")
{
  fault_collector::FaultCollectorConfig cfg;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 0.0);

  REQUIRE(core.consume_disarm_edge());
  REQUIRE_FALSE(core.consume_disarm_edge());
  REQUIRE_FALSE(core.consume_disarm_edge());
}

TEST_CASE(
  "With latch == false, the disarm edge re-arms after the fault expires and a new fault fires again",
  "[fault_collector_core][disarm_edge][expiry]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.fault_timeout_s = 5.0;
  cfg.latch = false;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 0.0);
  REQUIRE(core.consume_disarm_edge());
  REQUIRE_FALSE(core.consume_disarm_edge());

  core.expire(5.1);
  REQUIRE_FALSE(core.should_disarm());
  REQUIRE_FALSE(core.consume_disarm_edge());

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 6.0);
  REQUIRE(core.consume_disarm_edge());
  REQUIRE_FALSE(core.consume_disarm_edge());
}

TEST_CASE("clear_latch() cannot release a still-active fault", "[fault_collector_core][disarm_edge][latch]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.latch = true;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 0.0);
  REQUIRE(core.consume_disarm_edge());
  REQUIRE_FALSE(core.consume_disarm_edge());

  // clear_latch() drops the latches but not the fault table. Because the critical fault is
  // still active, both tiers re-trip on the very next evaluation and the edge fires again:
  // clearing faults is not an override, and the way out of the lockout is fixing the fault.
  core.clear_latch();
  REQUIRE(core.should_disarm());
  REQUIRE(core.should_brake());
  REQUIRE(core.disarm_required());
  REQUIRE(core.consume_disarm_edge());
  REQUIRE_FALSE(core.consume_disarm_edge());

  // Only once the underlying fault is gone does clearing the latch actually release.
  core.expire(100.0);
  core.clear_latch();
  REQUIRE_FALSE(core.should_disarm());
  REQUIRE_FALSE(core.should_brake());
  REQUIRE_FALSE(core.disarm_required());
}

// ---------------------------------------------------------------------------
// Re-engage lockout (disarm_required)
// ---------------------------------------------------------------------------

TEST_CASE("disarm_required() holds while a critical fault is active", "[fault_collector_core][lockout]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.latch = true;
  fault_collector::FaultCollectorCore core(cfg);

  REQUIRE_FALSE(core.disarm_required());

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 0.0);
  REQUIRE(core.disarm_required());

  // Latched: still required even once the fault stops being reported. This is what keeps an
  // operator from arming back out of a fault that was live moments ago.
  core.expire(100.0);
  REQUIRE(core.active_faults().empty());
  REQUIRE(core.disarm_required());
}

TEST_CASE("disarm_required() clears with the fault when latch == false", "[fault_collector_core][lockout]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.latch = false;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 0.0);
  REQUIRE(core.disarm_required());

  core.expire(100.0);
  REQUIRE_FALSE(core.disarm_required());
}

TEST_CASE("disarm_required() is non-mutating", "[fault_collector_core][lockout]")
{
  fault_collector::FaultCollectorConfig cfg;
  cfg.latch = true;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kCritical, "brakes", "actuator fault"), 0.0);

  // The node polls this on every is_armed transition, so it must neither set the latch nor
  // consume the edge no matter how often it is called.
  for (int i = 0; i < 10; ++i) {
    REQUIRE(core.disarm_required());
  }
  REQUIRE(core.consume_disarm_edge());  // edge still unconsumed despite the polling above

  core.expire(100.0);
  core.clear_latch();
  for (int i = 0; i < 10; ++i) {
    REQUIRE_FALSE(core.disarm_required());
  }
  REQUIRE_FALSE(core.should_disarm());  // polling did not latch anything
}

TEST_CASE("A SEVERE fault does not trigger the disarm lockout", "[fault_collector_core][lockout][tiers]")
{
  fault_collector::FaultCollectorConfig cfg;  // disarm_level=1, brake_level=2
  cfg.latch = true;
  fault_collector::FaultCollectorCore core(cfg);

  core.report(make_record(fault_collector::kSevere, "localization", "covariance diverging"), 0.0);

  REQUIRE(core.should_brake());
  REQUIRE_FALSE(core.should_disarm());
  REQUIRE_FALSE(core.disarm_required());
  REQUIRE_FALSE(core.consume_disarm_edge());
}

}  // namespace wato
