#include <algorithm>

#include "ars_pointcloud_filter.hpp"


// Filter Template

radar_msgs::msg::RadarPacket ARSPointCloudFilter::main_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double threshold, double filter_type)
  {
    radar_msgs::msg::RadarPacket main_filtered_ars;
    for (auto detection : unfiltered_ars->detections)
    {
      if(detection.{filter_type?} < threshold)
      {
        // Push filtered point/detection into an array
        main_filtered_ars.detections.push_back(detection);
      }
    }
    return main_filtered_ars;
  }

// SNR Filter
radar_msgs::msg::RadarPacket ARSPointCloudFilter::snr_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double snr_threshold)
{
  //filter out only nearscan packets
  if (unfiltered_ars->event_id == 12)
  {
    radar_msgs::msg::RadarPacket filtered_ars = main_filter(unfiltered_ars, snr_threshold, snr)
    return filtered_ars;
  }
}

// Azimuth Angle Filter
radar_msgs::msg::RadarPacket ARSPointCloudFilter::azimuth_angle_filter(
  const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double AzAng0_threshold)
{
  //filter out only nearscan packets
  if (unfiltered_ars->event_id == 12)
  {
    radar_msgs::msg::RadarPacket filtered_ars = main_filter(unfiltered_ars, AzAng0_threshold, az_ang0)
    return filtered_ars;
  }
}




/// ORIGINAL IMPLEMENTATION
// // SNR Filter
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::snr_filter(
//   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double snr_threshold)
// {
//   radar_msgs::msg::RadarPacket filtered_ars;
//   for (auto detection : unfiltered_ars->detections)
//   {
//     if(detection.snr < snr_threshold)
//     {
//      // Push filtered point/detection into an array
//      filtered_ars.detections.push_back(detection);
//     }
//   }
//   return filtered_ars;
// }

// // Azimuth Angle Filter
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::azimuth_angle_filter(
//   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double AzAng0_threshold)
// {
//   radar_msgs::msg::RadarPacket filtered_ars;
//   for (auto detection : unfiltered_ars->detections)
//   {
//     if(abs(detection.az_ang0) < abs(AzAng0_threshold))
//     {
//         // Point angles are within the defined constraints
//         filtered_ars.detections.push_back(detection);
//     }
//   }
//   return filtered_ars;
// }

// // Range Filter
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::range_filter(
//     const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double range_threshold)
// {
//     radar_msgs::msg::RadarPacket filtered_ars;
//     for (auto detection : unfiltered_ars->detections)
//     {
//         if(detection.range < range_threshold)
//         {
//             // an array of filtered objects
//             filtered_ars.detections.push_back(detection);
//         }
//     }
//     return filtered_ars;
// }

// // Relative Radial Velocity
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::vrel_rad_filter(
//     const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double vrel_rad_threshold)
// {
//     radar_msgs::msg::RadarPacket filtered_ars;
//     for (auto detection : unfiltered_ars->detections)
//     {
//         if(detection.vrel_rad < vrel_rad_threshold)
//         {
//             // an array of filtered objects
//             filtered_ars.detections.push_back(detection);
//         }
//     }
//     return filtered_ars;
// }

// // Elevation Angle
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::el_ang_filter(
//     const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double el_ang_threshold)
// {
//     radar_msgs::msg::RadarPacket filtered_ars;
//     for (auto detection : unfiltered_ars->detections)
//     {
//         if(detection.el_ang < el_ang_threshold)
//         {
//             // an array of filtered objects
//             filtered_ars.detections.push_back(detection);
//         }
//     }
//     return filtered_ars;
// }

// // Radar Cross Section
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::rcs_filter(
//     const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double rcs_threshold)
// {
//     radar_msgs::msg::RadarPacket filtered_ars;
//     for (auto detection : unfiltered_ars->detections)
//     {
//         if(detection.rcs0 < rcs_threshold)
//         {
//             // an array of filtered objects
//             filtered_ars.detections.push_back(detection);
//         }
//     }
//     return filtered_ars;
// }

