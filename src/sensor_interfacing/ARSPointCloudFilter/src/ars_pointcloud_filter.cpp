#include <algorithm>

#include "ars_pointcloud_filter.hpp"


//Packets will depend on which node is selected to run (could be carla packets or ars radar packets)

void ARSPointCloudFilter :: snr_filter(const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars_left,
                                       const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars_right)
{   
    //create a new filtered message (based on UnfilteredRadarPacket message type (will be changed))
    radar_msgs::msg::RadarPacket::SharedPtr filtered_left;
    double threshold = 12.0;
    for (auto detection:unfiltered_ars_left->Detections)
    {
        // -> or dot operator?
        if(detection.snr < threshold)
        {
            //push filtered point somewhere
        }
    }
    return filtered_packets
}

void ARSPointCloudFilter::radar_velocity_filter()
{


    

}

void ARSPointCloudFilter::azimuth_angle_filter()
{

}
