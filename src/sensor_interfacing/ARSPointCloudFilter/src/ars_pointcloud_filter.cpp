#include <algorithm>

#include "ars_pointcloud_filter.hpp"


// unfiltered left and right ars (how is this combined)?
radar_msgs::msg::RadarPacket ARSPointCloudFilter::snr_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                                       double snr_threshold)
{   
    radar_msgs::msg::RadarPacket filtered_ars;
    for (auto detection:unfiltered_ars->detections)
    {
        if(detection.snr < snr_threshold)
        {
            //an array of filtered objects
            filtered_ars.detections.push_back(detection);
        }
    }
    return filtered_ars;
}

radar_msgs::msg::RadarPacket ARSPointCloudFilter::azimuth_angle_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                                                double AzAng0_threshold, double AzAng1_threshold)           
{
    radar_msgs::msg::RadarPacket filtered_ars;
    for (auto detection:unfiltered_ars->detections)
    {
        if(abs(detection.az_ang0) < abs(AzAng0_threshold) && detection.az_ang1 < abs(AzAng1_threshold))
        {
            //point angles are within the defined constraints
            filtered_ars.detections.push_back(detection);
        }
    }
    return filtered_ars;

}

radar_msgs::msg::RadarPacket ARSPointCloudFilter::range_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                                       double range_threshold)
{
    radar_msgs::msg::RadarPacket filtered_ars;
    for (auto detection:unfiltered_ars->detections)
    {
        if(detection.range < range_threshold)
        {
            //an array of filtered objects
            filtered_ars.detections.push_back(detection);
        }
    }
    return filtered_ars;
}
