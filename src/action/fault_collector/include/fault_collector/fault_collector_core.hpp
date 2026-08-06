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
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace fault_collector
{

/// Severity levels mirroring wato_fault_msgs/msg/Fault. LOWER number == MORE severe.
static constexpr uint8_t kCritical = 1;  ///< Unsafe to continue driving. Disengage autonomy now.
static constexpr uint8_t kSevere = 2;  ///< Major degradation; autonomy may need to hand back control.
static constexpr uint8_t kModerate = 3;  ///< Degraded but driveable.
static constexpr uint8_t kMild = 4;  ///< Informational / nuisance.

/**
 * @brief Returns a human-readable name for a fault severity level.
 * @param level Severity level (1..4). Any other value returns "UNKNOWN".
 * @return Severity name, e.g. "CRITICAL".
 */
std::string severity_name(uint8_t level);

/**
 * @struct FaultCollectorConfig
 * @brief Tunable behaviour of the fault collector core.
 *
 * Faults drive two independent response tiers, both keyed off severity level:
 *   - Tier A (disarm): level <= disarm_level. The real vehicle authority
 *     (`oscc_interfacing`) is instructed to disarm via a one-shot service call.
 *   - Tier B (brake): level <= brake_level. `ackermann_mux` is told to emit its
 *     emergency command while still armed.
 * `disarm_level` must be <= `brake_level` -- disarm implies brake, never the reverse.
 */
struct FaultCollectorConfig
{
  double fault_timeout_s{5.0};  ///< Faults not re-reported within this window expire.
  uint8_t disarm_level{1};  ///< Tier A: faults at or below this level request a vehicle disarm.
  uint8_t brake_level{2};  ///< Tier B: faults at or below this level request an emergency brake.
  bool latch{true};  ///< Once a tier trips, stay tripped until explicitly cleared.
  std::array<uint8_t, 4> diagnostic_level_map{2, 2, 1, 0};  ///< Fault level 1..4 -> diagnostic_msgs level.

  /**
   * @brief Validates the tier ordering invariant.
   * @return True iff `disarm_level <= brake_level` (disarm is at least as strict as brake).
   */
  bool is_valid() const
  {
    return disarm_level <= brake_level;
  }
};

/**
 * @struct FaultRecord
 * @brief A single fault report handed to the core, decoupled from the ROS message type.
 */
struct FaultRecord
{
  uint8_t level{0};  ///< Severity level (1..4). See kCritical..kMild.
  std::string source;  ///< Where the fault came from -- node name, subsystem, or hardware id.
  std::string message;  ///< Human-readable description of what went wrong.
  std::string id;  ///< Optional stable identifier within `source`. Empty -> `message` is used.
  bool active{true};  ///< True = raise/refresh the fault. False = explicitly clear it.
};

/**
 * @struct ActiveFault
 * @brief Table entry tracking a currently-active fault.
 */
struct ActiveFault
{
  uint8_t level{0};  ///< Severity level (1..4).
  std::string source;  ///< Origin of the fault.
  std::string id;  ///< Stable identifier within `source` (may be empty).
  std::string message;  ///< Most recently reported human-readable description.
  double first_seen_s{0.0};  ///< Time (seconds) this fault was first reported.
  double last_seen_s{0.0};  ///< Time (seconds) this fault was most recently (re)reported.
  uint64_t count{0};  ///< Number of times this fault has been reported.
};

/**
 * @brief Outcome of a call to FaultCollectorCore::report, used by the node to decide whether to log a warning.
 */
enum class ReportResult
{
  kAccepted,  ///< Fault was inserted or refreshed in the table.
  kCleared,  ///< Fault was explicitly cleared (active == false) and removed from the table.
  kRejectedInvalidLevel,  ///< Report was dropped because `level` was not in 1..4.
  kRejectedEmptySource,  ///< Report was dropped because `source` was empty.
};

/**
 * @class FaultCollectorCore
 * @brief Pure logic fault table and two-tier response arbitration, with no ROS dependencies.
 *
 * Owns the table of currently-active faults and decides two independent things every tick:
 *   - Tier A / disarm: should the real vehicle authority (`oscc_interfacing`) be told to
 *     disarm? Exposed via should_disarm() (level) and consume_disarm_edge() (one-shot trigger).
 *   - Tier B / brake: should `ackermann_mux` be told to emit its emergency command while
 *     still armed? Exposed via should_brake() (continuous level, not edge-triggered).
 *
 * A persistent disarm-level fault must prevent autonomy from re-engaging. `oscc_interfacing`
 * owns arming and places no lockout on its own arm service, and it is deliberately left
 * unmodified by this package -- so the lockout is enforced entirely from the node built on top
 * of this core. The disarm service call itself is edge-triggered (consume_disarm_edge()) rather
 * than re-sent every tick, which keeps the service traffic bounded; the lockout comes instead
 * from disarm_required(), which the node polls whenever it observes `oscc_interfacing`'s
 * `is_armed` signal transition false->true. If a disarm-level fault is still active -- or, under
 * `latch: true` (the default), has been active at any point since the last clear_latch() -- the
 * node immediately re-issues the disarm. An operator therefore cannot arm their way out of a
 * live fault by pressing the arm button.
 *
 * Note that clear_latch() (exposed as `~/clear_faults`) is not an escape hatch either: it drops
 * the latches but not the fault table, so a fault that is genuinely still being reported simply
 * re-trips on the next evaluation. The way out of the lockout is fixing the fault.
 *
 * The core never reads the clock itself -- callers pass `now_s` explicitly -- which makes it
 * trivially unit-testable without spinning a node.
 *
 * Thread-safety is the caller's responsibility (the node only calls this from single-threaded
 * executor callbacks), so no internal mutex is used here.
 */
class FaultCollectorCore
{
public:
  /**
   * @brief Constructs the core with a fixed configuration.
   * @param config Behaviour configuration (timeouts, tier thresholds, latch behaviour).
   */
  explicit FaultCollectorCore(const FaultCollectorConfig & config);

  /**
   * @brief Inserts, refreshes, or clears a fault in the table.
   *
   * Faults are keyed by `source + "|" + (id.empty() ? message : id)`. A report that matches
   * an existing key bumps `count` and refreshes `last_seen_s` while preserving `first_seen_s`.
   * A report with `active == false` erases the matching entry instead.
   *
   * @param record The incoming fault report.
   * @param now_s Current time in seconds, supplied by the caller.
   * @return ReportResult describing what happened, so the node can log invalid reports.
   */
  ReportResult report(const FaultRecord & record, double now_s);

  /**
   * @brief Drops any active fault that has not been re-reported within `fault_timeout_s`.
   * @param now_s Current time in seconds, supplied by the caller.
   */
  void expire(double now_s);

  /**
   * @brief Returns the most severe (numerically lowest) level among active faults.
   * @return The worst active level, or std::nullopt if there are no active faults.
   */
  std::optional<uint8_t> worst_level() const;

  /**
   * @brief Determines whether Tier A (disarm) should be requested.
   *
   * True if any active fault has `level <= disarm_level`. If `latch` is enabled, once this
   * returns true it keeps returning true -- even after the offending fault clears or expires
   * -- until clear_latch() is called. This method is idempotent within a tick and safe to
   * call as often as needed (e.g. once for diagnostics, once via consume_disarm_edge()).
   *
   * @return True if a vehicle disarm is currently requested.
   */
  bool should_disarm();

  /**
   * @brief Determines whether Tier B (brake) should be requested.
   *
   * True if any active fault has `level <= brake_level`. Latches independently of
   * should_disarm() under the same `latch` setting. Unlike the disarm action, this is a
   * continuous, level-triggered signal -- `ackermann_mux` is expected to hold the emergency
   * command for as long as this returns true, not react to an edge.
   *
   * @return True if the emergency brake should currently be asserted.
   */
  bool should_brake();

  /**
   * @brief Consumes the rising edge of the disarm condition, for one-shot service calls.
   *
   * Internally evaluates the same condition as should_disarm() (and mutates the same latch),
   * but only returns true once per rising edge: the first call after the disarm condition
   * becomes true returns true, every subsequent call while it remains true returns false.
   * It re-arms (can return true again) once the condition clears -- which, under `latch:
   * true`, only happens via clear_latch(), and under `latch: false` happens as soon as the
   * triggering fault(s) clear or expire.
   *
   * This is what makes the disarm service call a single, edge-triggered action instead of a
   * repeating one: see the class-level comment for why that matters for operator authority.
   * There is no companion method that re-fires on a vehicle re-arm event, and none should be
   * added -- that would reintroduce the re-arm lockout this design explicitly avoids.
   *
   * @return True exactly once per rising edge into "disarm requested".
   */
  bool consume_disarm_edge();

  /**
   * @brief Non-mutating query of the current Tier A (disarm) condition.
   *
   * True if a disarm-level fault is active, or if the disarm latch is already set. Unlike
   * should_disarm() and consume_disarm_edge(), this has NO side effects: it neither sets the
   * latch nor consumes the edge. That matters because the node polls it from its `is_armed`
   * subscription on every arm-state transition to decide whether an operator's re-arm must be
   * undone — a query that must be safe to call arbitrarily often without perturbing the edge
   * detector.
   *
   * @return True if a vehicle disarm is currently warranted.
   */
  bool disarm_required() const;

  /**
   * @brief Releases both latches and resets the disarm edge detector so it can fire again.
   *
   * This is NOT an override: it drops the latches but not the fault table, so any fault still
   * active immediately re-trips its tier on the next evaluation.
   */
  void clear_latch();

  /**
   * @brief Drops every active fault from the table. Does not affect the latches.
   */
  void clear_all();

  /**
   * @brief Const accessor for the active fault table, in deterministic (sorted-by-key) order.
   * @return Map of fault key -> ActiveFault.
   */
  const std::map<std::string, ActiveFault> & active_faults() const;

private:
  /**
   * @brief Builds the table key for a fault report.
   * @param source Fault source.
   * @param id Optional stable identifier.
   * @param message Fault message, used as the identifier when `id` is empty.
   * @return The combined table key.
   */
  static std::string make_key(const std::string & source, const std::string & id, const std::string & message);

  /**
   * @brief Checks whether any active fault is at or more severe than `level`.
   * @param level Threshold severity level.
   * @return True if `fault.level <= level` for at least one active fault.
   */
  bool any_fault_at_or_below(uint8_t level) const;

  FaultCollectorConfig config_;  ///< Behaviour configuration.
  std::map<std::string, ActiveFault> faults_;  ///< Active fault table, keyed for deterministic iteration.
  bool disarm_latched_{false};  ///< True once a disarm-triggering fault has latched.
  bool brake_latched_{false};  ///< True once a brake-triggering fault has latched.
  bool disarm_edge_seen_{false};  ///< True once the current disarm-requested period has been consumed.
};

}  // namespace fault_collector
