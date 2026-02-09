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

// dynamic_object_store.hpp

#ifndef BEHAVIOUR__DYNAMIC_OBJECT_STORE_HPP_
#define BEHAVIOUR__DYNAMIC_OBJECT_STORE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

#include "behaviour/utils/utils.hpp"
#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace behaviour
{
  // DynamicObjectStore
  // Thread-safe cache of dynamic objects with fast lookup indexes.
  //
  // Logic:
  // - Build an immutable `Snapshot` from each incoming world-object message.
  // - Classify each object into supported entity types (car, traffic light).
  // - Populate lookup indexes by object id and by lanelet id.
  // - Atomically replace the current snapshot under a mutex.
  //
  // Assumptions:
  // - Class prediction is read from `detection.results[hypothesis_index]`.
  // - Lanelet association is read from `lanelet_ahead.current_lanelet_id`.
  class DynamicObjectStore
  {
  public:
    using MessageT = world_model_msgs::msg::WorldObjectArray;
    using ObjectT = world_model_msgs::msg::WorldObject;
    using ObjectId = std::string;

    explicit DynamicObjectStore(int hypothesis_index = 0)
        : hypothesis_index_(static_cast<size_t>(hypothesis_index)) {}

    // Snapshot
    // Immutable indexed view of one dynamic-object message.
    struct Snapshot
    {
      MessageT::ConstSharedPtr objects_snapshot_;

      // lanelet_id -> indices in objects_snapshot_->objects
      std::unordered_map<int64_t, std::vector<size_t>> cars_by_lanelet_;

      // detection.id -> index in objects_snapshot_->objects
      std::unordered_map<ObjectId, size_t> cars_by_id_;
      std::unordered_map<ObjectId, size_t> traffic_lights_by_id_;

      // Returns a car object by detection id, or `nullptr` if not found.
      const ObjectT *getCarById(const ObjectId id) const
      {
        return getById(*this, cars_by_id_, id);
      }

      // Returns a traffic-light object by detection id, or `nullptr` if not found.
      const ObjectT *getTrafficLightById(const ObjectId id) const
      {
        return getById(*this, traffic_lights_by_id_, id);
      }

      // Returns all car objects currently indexed under the given lanelet id.
      std::vector<const ObjectT *> getCarsInLanelet(const int64_t lanelet_id) const
      {
        return getObjects(*this, cars_by_lanelet_, lanelet_id);
      }
    };

    // Rebuilds and publishes the latest immutable snapshot from a new message.
    void update(MessageT::ConstSharedPtr msg)
    {
      auto snap = std::make_shared<Snapshot>();
      snap->objects_snapshot_ = std::move(msg);

      if (!snap->objects_snapshot_)
      {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = std::move(snap);
        return;
      }

      const auto &objs = snap->objects_snapshot_->objects;

      snap->cars_by_lanelet_.reserve(objs.size());
      snap->cars_by_id_.reserve(objs.size());
      snap->traffic_lights_by_id_.reserve(objs.size());

      for (size_t i = 0; i < objs.size(); ++i)
      {
        const auto &o = objs[i];

        const int64_t lanelet_id = o.lanelet_ahead.current_lanelet_id;
        const std::string_view class_id = getBestClassId(o);
        const types::EntityType type = classify(class_id);

        if (type == types::EntityType::UNKNOWN)
        {
          continue;
        }

        const ObjectId &id = o.detection.id;

        switch (type)
        {
        case types::EntityType::CAR:
          snap->cars_by_id_[id] = i;
          if (lanelet_id != kInvalidLanelet)
          {
            snap->cars_by_lanelet_[lanelet_id].push_back(i);
          }
          break;

        case types::EntityType::TRAFFIC_LIGHT:
          snap->traffic_lights_by_id_[id] = i;
          break;

        default:
          break;
        }
      }

      {
        std::lock_guard<std::mutex> lock(mutex_);
        snapshot_ = std::move(snap);
      }
    }

    // Returns the latest immutable snapshot (may be null before first update).
    std::shared_ptr<const Snapshot> snapshot() const
    {
      std::lock_guard<std::mutex> lock(mutex_);
      return snapshot_;
    }

  private:
    static constexpr int64_t kInvalidLanelet = -1;

    // Maps classifier output labels to internal entity types.
    static types::EntityType classify(const std::string_view class_id)
    {
      if (class_id == "car" || class_id == "vehicle" || class_id == "truck")
      {
        return types::EntityType::CAR;
      }
      if (class_id == "red" || class_id == "green" || class_id == "yellow" || class_id == "traffic_light")
      {
        return types::EntityType::TRAFFIC_LIGHT;
      }
      return types::EntityType::UNKNOWN;
    }

    // Returns the class id from detection results at the configured hypothesis index, or empty view.
    std::string_view getBestClassId(const ObjectT &o) const
    {
      if (o.detection.results.empty())
        return {};
      if (hypothesis_index_ >= o.detection.results.size())
        return {};
      return o.detection.results[hypothesis_index_].hypothesis.class_id;
    }

    // Returns the mapped index list for a key, or a shared empty list.
    template <typename MapT, typename KeyT>
    static const std::vector<size_t> &getIndices(const MapT &map, const KeyT &key)
    {
      static const std::vector<size_t> empty_idxs{};
      auto it = map.find(key);
      return (it == map.end()) ? empty_idxs : it->second;
    }

    // Materializes object pointers for all indices mapped from a key.
    template <typename MapT, typename KeyT>
    static std::vector<const ObjectT *> getObjects(const Snapshot &snap, const MapT &lanelet_map, const KeyT &key)
    {
      std::vector<const ObjectT *> out;
      if (!snap.objects_snapshot_)
        return out;

      const auto &idxs = getIndices(lanelet_map, key);
      out.reserve(idxs.size());
      for (size_t idx : idxs)
      {
        if (idx < snap.objects_snapshot_->objects.size())
        {
          out.push_back(&snap.objects_snapshot_->objects[idx]);
        }
      }
      return out;
    }

    // Returns an object pointer by id-map lookup, or `nullptr` if missing/invalid.
    template <typename MapT, typename KeyT>
    static const ObjectT *getById(const Snapshot &snap, const MapT &id_map, const KeyT &key)
    {
      if (!snap.objects_snapshot_)
        return nullptr;

      auto it = id_map.find(key);
      if (it == id_map.end())
        return nullptr;

      const size_t idx = it->second;
      if (idx >= snap.objects_snapshot_->objects.size())
        return nullptr;

      return &snap.objects_snapshot_->objects[idx];
    }

    mutable std::mutex mutex_;
    std::shared_ptr<const Snapshot> snapshot_;
    size_t hypothesis_index_{0};
  };

} // namespace behaviour

#endif // BEHAVIOUR__DYNAMIC_OBJECT_STORE_HPP_
