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

#ifndef WORLD_MODEL__INTERFACES__WORKERS__CLEANUP_WORKER_HPP_
#define WORLD_MODEL__INTERFACES__WORKERS__CLEANUP_WORKER_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <thread>

#include "rclcpp/clock.hpp"
#include "world_model/interfaces/interface_base.hpp"

namespace world_model
{

/**
 * @brief Background worker that prunes stale entities from buffers.
 *
 * Periodically removes entities that haven't been updated within
 * the configured timeout.
 */
class CleanupWorker : public InterfaceBase
{
public:
  CleanupWorker(
    WorldState * world_state,
    rclcpp::Clock::SharedPtr clock,
    std::chrono::milliseconds interval,
    double entity_timeout_sec,
    double traffic_light_timeout_sec)
  : world_state_(world_state)
  , clock_(clock)
  , interval_(interval)
  , entity_timeout_sec_(entity_timeout_sec)
  , traffic_light_timeout_sec_(traffic_light_timeout_sec)
  {}

  ~CleanupWorker() override
  {
    stop();
  }

  // Non-copyable, non-movable
  CleanupWorker(const CleanupWorker &) = delete;
  CleanupWorker & operator=(const CleanupWorker &) = delete;
  CleanupWorker(CleanupWorker &&) = delete;
  CleanupWorker & operator=(CleanupWorker &&) = delete;

  void activate() override
  {
    if (running_.load()) {
      return;
    }
    running_.store(true);
    thread_ = std::thread(&CleanupWorker::run, this);
  }

  void deactivate() override
  {
    stop();
  }

private:
  void stop()
  {
    if (!running_.load()) {
      return;
    }
    {
      std::lock_guard lock(mutex_);
      running_.store(false);
    }
    cv_.notify_all();
    if (thread_.joinable()) {
      thread_.join();
    }
  }

  void run()
  {
    while (running_.load()) {
      doWork();

      std::unique_lock lock(mutex_);
      cv_.wait_for(lock, interval_, [this]() { return !running_.load(); });
    }
  }

  void doWork()
  {
    auto now = clock_->now();

    auto should_prune_entity = [&now, this](const auto & entity) {
      if (entity.empty()) {
        return true;
      }
      return (now - entity.timestamp()).seconds() > entity_timeout_sec_;
    };

    world_state_.buffer<Car>().prune(should_prune_entity);
    world_state_.buffer<Human>().prune(should_prune_entity);
    world_state_.buffer<Bicycle>().prune(should_prune_entity);
    world_state_.buffer<Motorcycle>().prune(should_prune_entity);

    // Traffic lights have separate timeout
    world_state_.buffer<TrafficLight>().prune([&now, this](const TrafficLight & tl) {
      if (tl.empty()) {
        return true;
      }
      return (now - tl.timestamp()).seconds() > traffic_light_timeout_sec_;
    });
  }

  WorldStateWriter world_state_;
  rclcpp::Clock::SharedPtr clock_;
  std::chrono::milliseconds interval_;
  double entity_timeout_sec_;
  double traffic_light_timeout_sec_;

  std::thread thread_;
  std::atomic<bool> running_{false};
  std::mutex mutex_;
  std::condition_variable cv_;
};

}  // namespace world_model

#endif  // WORLD_MODEL__INTERFACES__WORKERS__CLEANUP_WORKER_HPP_
