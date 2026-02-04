// area_occupancy_store.hpp

#ifndef BEHAVIOUR__AREA_OCCUPANCY_STORE_HPP_
#define BEHAVIOUR__AREA_OCCUPANCY_STORE_HPP_

#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "world_model_msgs/msg/area_occupancy.hpp"
#include "world_model_msgs/msg/world_object.hpp"

namespace behaviour
{

    class AreaOccupancyStore
    {
    public:
        using MessageT = world_model_msgs::msg::AreaOccupancy;
        using ObjectT = world_model_msgs::msg::WorldObject;

        struct Snapshot
        {
            MessageT::ConstSharedPtr msg_;

            // area_name -> index into msg_->areas
            std::unordered_map<std::string, size_t> area_index_;

            bool isOccupied(const std::string &name) const
            {
                const auto *a = getArea(name);
                return a ? a->is_occupied : false;
            }

            const std::vector<ObjectT> *objects(const std::string &name) const
            {
                const auto *a = getArea(name);
                return a ? &a->objects : nullptr;
            }

        private:
            const world_model_msgs::msg::AreaOccupancyInfo *getArea(const std::string &name) const
            {
                if (!msg_)
                    return nullptr;

                // find the index
                auto it = area_index_.find(name);
                if (it == area_index_.end())
                    return nullptr;

                // get the area info by index
                const size_t idx = it->second;
                if (idx >= msg_->areas.size())
                    return nullptr;
                return &msg_->areas[idx];
            }
        };

        void update(MessageT::ConstSharedPtr msg)
        {
            auto snap = std::make_shared<Snapshot>();
            snap->msg_ = std::move(msg);

            if (snap->msg_)
            {
                snap->area_index_.reserve(snap->msg_->areas.size());
                for (size_t i = 0; i < snap->msg_->areas.size(); ++i)
                {
                    snap->area_index_.emplace(snap->msg_->areas[i].name, i);
                }
            }

            std::lock_guard<std::mutex> lock(mutex_);
            snapshot_ = std::move(snap);
        }

        std::shared_ptr<const Snapshot> snapshot() const
        {
            std::lock_guard<std::mutex> lock(mutex_);
            return snapshot_;
        }

    private:
        mutable std::mutex mutex_;
        std::shared_ptr<const Snapshot> snapshot_;
    };

} // namespace behaviour

#endif // BEHAVIOUR__AREA_OCCUPANCY_STORE_HPP_