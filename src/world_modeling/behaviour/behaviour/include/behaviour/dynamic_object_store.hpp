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
#include <vector>

#include "behaviour/utils/utils.hpp"

#include "world_model_msgs/msg/world_object.hpp"
#include "world_model_msgs/msg/world_object_array.hpp"

namespace behaviour
{
  class DynamicObjectStore
  {
  public:
    using MessageT = world_model_msgs::msg::WorldObjectArray;
    using ObjectT = world_model_msgs::msg::WorldObject;
    using ObjectId = std::string;

    struct Snapshot
    {
      MessageT::ConstSharedPtr objects_snapshot_;

      // lanelet_id -> indices in objects_snapshot_->objects
      std::unordered_map<int64_t, std::vector<size_t>> cars_by_lanelet_;

      // detection.id -> index in objects_snapshot_->objects
      std::unordered_map<ObjectId, size_t> cars_by_id_;
      std::unordered_map<ObjectId, size_t> traffic_lights_by_id_;

      const ObjectT *getCarById(const ObjectId id) const
      {
        return getById(*this, cars_by_id_, id);
      }

      const ObjectT *getTrafficLightById(const ObjectId id) const
      {
        return getById(*this, traffic_lights_by_id_, id);
      }

      std::vector<const ObjectT *> getCarsInLanelet(const int64_t lanelet_id) const
      {
        return getObjects(*this, cars_by_lanelet_, lanelet_id);
      }
    };

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

        // Assumption: detection.id is always defined
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

    std::shared_ptr<const Snapshot> snapshot() const
    {
      std::lock_guard<std::mutex> lock(mutex_);
      return snapshot_;
    }

  private:
    static constexpr int64_t kInvalidLanelet = -1;

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

    static std::string_view getBestClassId(const ObjectT &o)
    {
      if (o.detection.results.empty())
        return {};
      return o.detection.results[0].hypothesis.class_id;
    }

    template <typename MapT, typename KeyT>
    static const std::vector<size_t> &getIndices(const MapT &map, const KeyT &key)
    {
      static const std::vector<size_t> empty_idxs{};
      auto it = map.find(key);
      return (it == map.end()) ? empty_idxs : it->second;
    }

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
  };

} // namespace behaviour

#endif // BEHAVIOUR__DYNAMIC_OBJECT_STORE_HPP_
