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

#ifndef BEHAVIOUR__UTILS__DYNAMIC_OBJECT_STORE_HPP_
#define BEHAVIOUR__UTILS__DYNAMIC_OBJECT_STORE_HPP_

#include <memory>
#include <mutex>  // Back to standard mutex
#include <unordered_map>
#include <vector>

#include "world_model_msgs/msg/dynamic_object.hpp"
#include "world_model_msgs/msg/dynamic_object_array.hpp"

namespace behaviour
{
class DynamicObjectStore
{
public:
  using DynamicObject = world_model_msgs::msg::DynamicObject;
  using DynamicObjectPtr = std::shared_ptr<DynamicObject>;
  using DynamicObjectArray = world_model_msgs::msg::DynamicObjectArray;

  /**
     * @brief Update the store with a new array of objects.
     */
  void update(const DynamicObjectArray::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);  // Simple exclusive lock
    objects_by_id_.clear();
    objects_by_lanelet_.clear();

    for (const auto & obj : msg->objects) {
      objects_by_id_[obj.id] = obj;
      if (obj.lanelet_id != -1) {
        objects_by_lanelet_[obj.lanelet_id].push_back(obj.id);
      }
    }
  }

  /**
     * @brief Get all objects on a lanelet.
     */
  std::vector<DynamicObject> getObjectsOnLanelet(int64_t lanelet_id) const
  {
    std::lock_guard lock(mutex_);
    std::vector<DynamicObject> result;

    if (objects_by_lanelet_.count(lanelet_id)) {
      for (int64_t id : objects_by_lanelet_.at(lanelet_id)) {
        result.push_back(objects_by_id_.at(id));  // Copies object into the vector
      }
    }
    return result;
  }

  /**
     * @brief Get an object by its ID.
     * @return The object if found. Returns an empty object or throws if not.
     */
  DynamicObject getObjectById(int64_t id) const
  {
    std::lock_guard lock(mutex_);
    // We return a copy. The compiler will likely optimize this away (NRVO).
    return objects_by_id_.at(id);
  }

  /**
     * @brief Get all objects in the store.
     */
  std::vector<DynamicObject> getAllObjects() const
  {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<DynamicObject> result;
    result.reserve(objects_by_id_.size());
    for (const auto & [id, obj] : objects_by_id_) {
      result.push_back(obj);
    }
    return result;
  }

private:
  mutable std::mutex mutex_;  // Simple mutex
  std::unordered_map<int64_t, world_model_msgs::msg::DynamicObject> objects_by_id_;
  std::unordered_map<int64_t, std::vector<int64_t>> objects_by_lanelet_;
};

}  // namespace behaviour

#endif
