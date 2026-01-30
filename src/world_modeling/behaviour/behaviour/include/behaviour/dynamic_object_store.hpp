// dynamic_object_store.hpp
//
// Thread-safe DynamicObjectStore with snapshot semantics.
//
// - Subscriber thread calls: store.update(msg)
// - Tick thread sets BB:      bb["snap.dyn"] = store.snapshot()
// - BT nodes read snap.dyn and call snapshot methods.
//
// Returned object pointers remain valid as long as the Snapshot shared_ptr
// is kept alive (because Snapshot holds objects_snapshot_).

#ifndef BEHAVIOUR__DYNAMIC_OBJECT_STORE_HPP_
#define BEHAVIOUR__DYNAMIC_OBJECT_STORE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "world_model_msgs/msg/dynamic_object.hpp"
#include "world_model_msgs/msg/dynamic_object_array.hpp"

namespace behaviour
{

class DynamicObjectStore
{
public:
  struct Snapshot
  {
    world_model_msgs::msg::DynamicObjectArray::ConstSharedPtr objects_snapshot_;

    // lanelet_id -> indices -> objects_snapshot_->objects[index]
    std::unordered_map<int64_t, std::vector<size_t>> cars_by_lanelet_;

    // object_id -> index -> objects_snapshot_->objects[index]
    std::unordered_map<int64_t, size_t> cars_by_id_;

    const world_model_msgs::msg::DynamicObject* getCarsById(const int64_t id) const
    {
      if (!objects_snapshot_) return nullptr;

      auto it = cars_by_id_.find(id);
      if (it == cars_by_id_.end()) return nullptr;

      const size_t idx = it->second;
      if (idx >= objects_snapshot_->objects.size()) return nullptr;  // defensive

      return &objects_snapshot_->objects[idx];
    }

    // Returns indices (no copying of objects)
    const std::vector<size_t>& getCarIndicesInLanelet(const int64_t lanelet_id) const
    {
      static const std::vector<size_t> kEmpty{};
      auto it = cars_by_lanelet_.find(lanelet_id);
      return (it == cars_by_lanelet_.end()) ? kEmpty : it->second;
    }

    // Optional convenience: materialize pointers for callers
    std::vector<const world_model_msgs::msg::DynamicObject*> getCarsInLanelet(const int64_t lanelet_id) const
    {
      std::vector<const world_model_msgs::msg::DynamicObject*> out;
      if (!objects_snapshot_) return out;

      const auto& idxs = getCarIndicesInLanelet(lanelet_id);
      out.reserve(idxs.size());
      for (size_t idx : idxs) {
        if (idx < objects_snapshot_->objects.size()) {
          out.push_back(&objects_snapshot_->objects[idx]);
        }
      }
      return out;
    }
  };


  // Build and publish a new snapshot.
  void update(world_model_msgs::msg::DynamicObjectArray::ConstSharedPtr msg)
  {
    auto snap = std::make_shared<Snapshot>();
    snap->objects_snapshot_ = std::move(msg);

    if (!snap->objects_snapshot_) {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_ = std::move(snap);
      return;
    }

    const auto& objs = snap->objects_snapshot_->objects;
    snap->cars_by_lanelet_.reserve(objs.size());
    snap->cars_by_id_.reserve(objs.size());

    for (size_t i = 0; i < snap->objects_snapshot_->objects.size(); ++i) {
      const auto& o = snap->objects_snapshot_->objects[i];

      if (o.entity_type == world_model_msgs::msg::DynamicObject::TYPE_CAR) {
        snap->cars_by_id_[o.id] = i;
  
        if (o.lanelet_id != -1) {
          snap->cars_by_lanelet_[o.lanelet_id].push_back(i);
        }
      };
    }
    // Publish snapshot (swap under lock)
    {
      std::lock_guard<std::mutex> lock(mutex_);
      snapshot_ = std::move(snap);
    }
  }

  // Get the latest snapshot pointer (cheap shared_ptr copy).
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

#endif  // BEHAVIOUR__DYNAMIC_OBJECT_STORE_HPP_