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

#include "fault_collector/fault_collector_core.hpp"

#include <map>
#include <string>
#include <utility>

namespace fault_collector
{

std::string severity_name(uint8_t level)
{
  switch (level) {
    case kCritical:
      return "CRITICAL";
    case kSevere:
      return "SEVERE";
    case kModerate:
      return "MODERATE";
    case kMild:
      return "MILD";
    default:
      return "UNKNOWN";
  }
}

FaultCollectorCore::FaultCollectorCore(const FaultCollectorConfig & config)
: config_(config)
{}

std::string FaultCollectorCore::make_key(
  const std::string & source, const std::string & id, const std::string & message)
{
  return source + "|" + (id.empty() ? message : id);
}

ReportResult FaultCollectorCore::report(const FaultRecord & record, double now_s)
{
  if (record.source.empty()) {
    return ReportResult::kRejectedEmptySource;
  }

  if (record.level < kCritical || record.level > kMild) {
    return ReportResult::kRejectedInvalidLevel;
  }

  const std::string key = make_key(record.source, record.id, record.message);

  if (!record.active) {
    faults_.erase(key);
    return ReportResult::kCleared;
  }

  auto it = faults_.find(key);
  if (it == faults_.end()) {
    ActiveFault fault;
    fault.level = record.level;
    fault.source = record.source;
    fault.id = record.id;
    fault.message = record.message;
    fault.first_seen_s = now_s;
    fault.last_seen_s = now_s;
    fault.count = 1;
    faults_.emplace(key, std::move(fault));
  } else {
    it->second.level = record.level;
    it->second.message = record.message;
    it->second.last_seen_s = now_s;
    it->second.count += 1;
  }

  // Latch at report time rather than as a side effect of polling should_disarm()/should_brake().
  // The lockout must not depend on anyone having called a getter first: disarm_required() is
  // read from the is_armed callback, which can fire before the next diagnostics tick.
  if (config_.latch) {
    if (record.level <= config_.disarm_level) {
      disarm_latched_ = true;
    }
    if (record.level <= config_.brake_level) {
      brake_latched_ = true;
    }
  }

  return ReportResult::kAccepted;
}

void FaultCollectorCore::expire(double now_s)
{
  for (auto it = faults_.begin(); it != faults_.end();) {
    if (now_s - it->second.last_seen_s > config_.fault_timeout_s) {
      it = faults_.erase(it);
    } else {
      ++it;
    }
  }
}

std::optional<uint8_t> FaultCollectorCore::worst_level() const
{
  std::optional<uint8_t> worst;
  for (const auto & [key, fault] : faults_) {
    (void)key;
    if (!worst.has_value() || fault.level < *worst) {
      worst = fault.level;
    }
  }
  return worst;
}

bool FaultCollectorCore::any_fault_at_or_below(uint8_t level) const
{
  for (const auto & [key, fault] : faults_) {
    (void)key;
    if (fault.level <= level) {
      return true;
    }
  }
  return false;
}

bool FaultCollectorCore::should_disarm()
{
  const bool triggered = any_fault_at_or_below(config_.disarm_level);

  if (!config_.latch) {
    return triggered;
  }

  if (triggered) {
    disarm_latched_ = true;
  }
  return disarm_latched_;
}

bool FaultCollectorCore::should_brake()
{
  const bool triggered = any_fault_at_or_below(config_.brake_level);

  if (!config_.latch) {
    return triggered;
  }

  if (triggered) {
    brake_latched_ = true;
  }
  return brake_latched_;
}

bool FaultCollectorCore::disarm_required() const
{
  return disarm_latched_ || any_fault_at_or_below(config_.disarm_level);
}

bool FaultCollectorCore::consume_disarm_edge()
{
  const bool current = should_disarm();

  if (current && !disarm_edge_seen_) {
    disarm_edge_seen_ = true;
    return true;
  }

  if (!current) {
    // Condition cleared (only reachable with latch == false, or after clear_latch()).
    // Re-arm so the next rising edge fires again.
    disarm_edge_seen_ = false;
  }

  return false;
}

void FaultCollectorCore::clear_latch()
{
  disarm_latched_ = false;
  brake_latched_ = false;
  disarm_edge_seen_ = false;
}

void FaultCollectorCore::clear_all()
{
  faults_.clear();
}

const std::map<std::string, ActiveFault> & FaultCollectorCore::active_faults() const
{
  return faults_;
}

}  // namespace fault_collector
