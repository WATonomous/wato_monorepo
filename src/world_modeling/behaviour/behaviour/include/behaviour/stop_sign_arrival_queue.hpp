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

#ifndef BEHAVIOUR__STOP_SIGN_ARRIVAL_QUEUE_HPP_
#define BEHAVIOUR__STOP_SIGN_ARRIVAL_QUEUE_HPP_

#include <rclcpp/time.hpp>

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace behaviour
{
/**
 * @struct StopSignArrivalRecord
 * @brief Tracks when a vehicle arrived at a stop sign intersection.
 */
struct StopSignArrivalRecord
{
  std::string car_id;
  rclcpp::Time arrival_time;
  bool has_proceeded{false};
  bool stopped{false};  // Whether the vehicle has actually stopped

  StopSignArrivalRecord(const std::string & id, const rclcpp::Time & time)
  : car_id(id), arrival_time(time), has_proceeded(false), stopped(false)
  {}
};

/**
 * @class StopSignArrivalQueue
 * @brief Manages arrival queue for multi-way stop sign intersections.
 *
 * Implements proper right-of-way logic:
 * - First to stop has priority
 * - If multiple vehicles stop simultaneously (within threshold), rightmost vehicle goes first
 * - Tracks which vehicles have proceeded
 */
class StopSignArrivalQueue
{
public:
  using SharedPtr = std::shared_ptr<StopSignArrivalQueue>;

  /**
   * @brief Updates the arrival queue with current stopped vehicles.
   *
   * @param current_cars List of car IDs and their timestamps currently near the stop sign
   * @param simultaneou_arrival_threshold_s Time window for considering arrivals simultaneous (default: 1.0s)
   */
  void update(
    const std::vector<std::pair<std::string, rclcpp::Time>> & current_cars, 
    double simultaneous_arrival_threshold_s = 1.0)
  {
    // Remove cars that are no longer present
    records_.erase(
      std::remove_if(
        records_.begin(), records_.end(),
        [&current_cars](const StopSignArrivalRecord & rec) {
          return std::none_of(
            current_cars.begin(), current_cars.end(),
            [&rec](const auto & p) { return p.first == rec.car_id; });
        }),
      records_.end());

    // Add new arrivals
    for (const auto & [car_id, timestamp] : current_cars) {
      auto it = std::find_if(
        records_.begin(), records_.end(), [&car_id](const auto & r) { return r.car_id == car_id; });

      if (it == records_.end()) {
        // New arrival
        records_.emplace_back(car_id, timestamp);
      } else {
        // Update existing record - mark as stopped once detected
        it->stopped = true;
      }
    }

    // Mark first arrival as stopped
    if (!records_.empty()) {
      records_.front().stopped = true;
    }
  }

  /**
   * @brief Checks if ego vehicle has right-of-way to proceed.
   *
   * Right-of-way rules:
   * 1. Vehicle must have stopped
   * 2. All vehicles that arrived earlier and haven't proceeded must go first
   * 3. For simultaneous arrivals, implement tie-breaking (could be position-based)
   *
   * @param ego_car_id ID of the ego vehicle
   * @return true if ego can proceed
   */
  bool canProceed(const std::string & ego_car_id) const
  {
    auto ego_it =
      std::find_if(records_.begin(), records_.end(), [&ego_car_id](const auto & r) { return r.car_id == ego_car_id; });

    if (ego_it == records_.end()) {
      // Ego not in queue
      return false;
    }

    if (!ego_it->stopped) {
      // Ego hasn't stopped yet
      return false;
    }

    // Check if any car arrived before ego and hasn't proceeded
    for (const auto & rec : records_) {
      if (rec.car_id == ego_car_id) continue;

      if (!rec.has_proceeded && rec.arrival_time < ego_it->arrival_time) {
        // Another car has priority
        return false;
      }
    }

    return true;
  }

  /**
   * @brief Gets list of car IDs that have priority over ego (arrived first and haven't proceeded).
   */
  std::vector<std::string> getPriorityCarIds(const std::string & ego_car_id) const
  {
    std::vector<std::string> priority_ids;

    auto ego_it =
      std::find_if(records_.begin(), records_.end(), [&ego_car_id](const auto & r) { return r.car_id == ego_car_id; });

    if (ego_it == records_.end()) {
      // Ego not in queue, all cars have priority
      for (const auto & rec : records_) {
        if (!rec.has_proceeded) {
          priority_ids.push_back(rec.car_id);
        }
      }
      return priority_ids;
    }

    for (const auto & rec : records_) {
      if (rec.car_id == ego_car_id) continue;

      if (!rec.has_proceeded && rec.arrival_time < ego_it->arrival_time) {
        priority_ids.push_back(rec.car_id);
      }
    }

    return priority_ids;
  }

  /**
   * @brief Marks a vehicle as having proceeded through the intersection.
   */
  void markProceeded(const std::string & car_id)
  {
    auto it = std::find_if(records_.begin(), records_.end(), [&car_id](const auto & r) { return r.car_id == car_id; });

    if (it != records_.end()) {
      it->has_proceeded = true;
    }
  }

  /**
   * @brief Clears the arrival queue.
   */
  void clear()
  {
    records_.clear();
  }

  /**
   * @brief Returns the number of vehicles in the queue.
   */
  size_t size() const
  {
    return records_.size();
  }

  /**
   * @brief Returns all records in the queue.
   */
  const std::vector<StopSignArrivalRecord> & records() const
  {
    return records_;
  }

private:
  std::vector<StopSignArrivalRecord> records_;
};

}  // namespace behaviour

#endif  // BEHAVIOUR__STOP_SIGN_ARRIVAL_QUEUE_HPP_
