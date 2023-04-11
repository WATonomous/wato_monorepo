#include <string>
#include <vector>

#include "average_filter.hpp"

namespace world_modeling::hd_map
{
    AverageFilter::AverageFilter()
    {}

    sample_msgs::msg::FilteredArrayAverage AverageFilter::getAverage(const sample_msgs::msg::FilteredArray::SharedPtr msg) {
        sample_msgs::msg::FilteredArrayAverage avg = sample_msgs::msg::FilteredArrayAverage();
        avg.filtered_array = *msg;

        int count;

        for (auto packet : msg->packets) {
            count++;
            avg.avg_x += packet.pos_x;
            avg.avg_y += packet.pos_y;
            avg.avg_z += packet.pos_z;
        }

        avg.avg_x /= count;
        avg.avg_y /= count;
        avg.avg_z /= count; 

        return avg; 
    }
    
}  // namespace world_modeling::hd_map
