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

// area_occupancy_store.hpp

#ifndef BEHAVIOUR__AREA_OCCUPANCY_STORE_HPP_
#define BEHAVIOUR__AREA_OCCUPANCY_STORE_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "world_model_msgs/msg/area_occupancy.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{

// AreaOccupancyStore
// Thread-safe cache of latest area occupancy message with area-name indexing.
class AreaOccupancyStore
{
public:
  using MessageT = world_model_msgs::msg::AreaOccupancy;
  using ObjectT = world_model_msgs::msg::WorldObject;

  // Snapshot
  // Immutable view of one area occupancy message with fast area lookups.
  struct Snapshot
  {
    MessageT::ConstSharedPtr msg_;

    // area_name -> index into msg_->areas
    std::unordered_map<std::string, size_t> area_index_;

    // Returns whether the named area is occupied. Missing area returns false.
    bool isOccupied(const std::string & name) const
    {
      const auto * a = getArea(name);
      return a ? a->is_occupied : false;
    }

    // Returns pointer to objects in the named area, or nullptr when missing.
    const std::vector<ObjectT> * objects(const std::string & name) const
    {
      const auto * a = getArea(name);
      return a ? &a->objects : nullptr;
    }

  private:
    // Returns area info by name, or nullptr if snapshot/name/index is invalid.
    const world_model_msgs::msg::AreaOccupancyInfo * getArea(const std::string & name) const
    {
      if (!msg_) return nullptr;

      // find the index
      auto it = area_index_.find(name);
      if (it == area_index_.end()) return nullptr;

      // get the area info by index
      const size_t idx = it->second;
      if (idx >= msg_->areas.size()) return nullptr;
      return &msg_->areas[idx];
    }
  };

  // Rebuilds and publishes the latest immutable snapshot from a new message.
  void update(MessageT::ConstSharedPtr msg)
  {
    auto snap = std::make_shared<Snapshot>();
    snap->msg_ = std::move(msg);

    if (snap->msg_) {
      snap->area_index_.reserve(snap->msg_->areas.size());
      for (size_t i = 0; i < snap->msg_->areas.size(); ++i) {
        snap->area_index_.emplace(snap->msg_->areas[i].name, i);
      }
    }

    std::lock_guard<std::mutex> lock(mutex_);
    snapshot_ = std::move(snap);
  }

  // Returns the latest immutable snapshot (may be null before first update).
  std::shared_ptr<const Snapshot> snapshot() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return snapshot_;
  }

private:
  mutable std::mutex mutex_;
  std::shared_ptr<const Snapshot> snapshot_;
};

}  // namespace behaviour

#endif  // BEHAVIOUR__AREA_OCCUPANCY_STORE_HPP_
