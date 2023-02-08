#include <algorithm>

#include "ars_pointcloud_filter.hpp"


// unfiltered left and right ars (how is this combined)?
void ARSPointCloudFilter :: snr_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                                       float snr_threshold)
{   
    radar_msgs::msg::RadarPacket filtered_ars;
    for (auto detection:unfiltered_ars->Detections)
    {
        if(detection->SNR < snr_threshold)
        {
            //an array of filtered objects
            filtered_ars.Detections.push_back(detection);
        }
    }
    return filtered_ars;
}

void ARSPointCloudFilter::azimuth_angle_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                                                float AzAng0_threshold, float AzAng1_threshold)           
{
    radar_msgs::msg::RadarPacket filtered_ars;
    for (auto detection:unfiltered_ars->Detections)
    {
        if(abs(detection->AzAng0) < abs(AzAng0_threshold) && detection->AzAng1 < abs(AzAng1_threshold))
        {
            //point angles are within the defined constraints
            filtered_ars.Detections.push_back(detection);
        }
    }
    return filtered_ars;

}

void ARSPointCloudFilter::range_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
                                       float range_threshold)
{
    radar_msgs::msg::RadarPacket filtered_ars;
    for (auto detection:unfiltered_ars->Detections)
    {
        if(detection->Range < range_threshold)
        {
            //an array of filtered objects
            filtered_ars.Detections.push_back(detection);
        }
    }
    return filtered_ars;
}
