#include <algorithm>

#include "radar_pointcloud_filter.hpp"


//Packets will depend on which parameter is selected (could be carla packets or hardware packets)
void PointCloudFilter :: snr_filter
{
    RadarDetection snr_filter(const RadarDetection& packet)
    {
        RadarDetection unfiltered_kace
    }
}