#include <algorithm>

#include "ars_pointcloud_filter.hpp"

// SOLUTION 1: Via Conditions and ROS parameters

namespace filtering
{

ARSPointCloudFilter::ARSPointCloudFilter()
{

}

radar_msgs::msg::RadarPacket ARSPointCloudFilter::point_filter(
    const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars,
    double snr_threshold,
    double AzAng0_threshold,
    double range_threshold,
    double vrel_rad_threshold,
    double el_ang_threshold,
    double rcs_threshold)
    {
      radar_msgs::msg::RadarPacket filtered_ars;
      for (auto detection : unfiltered_ars->detections)
      {
        if(detection.snr < snr_threshold)
        {
          continue;
        }
        else if (detection.az_ang0 < AzAng0_threshold)
        {
          continue;
        }
        else if (detection.range < range_threshold)
        {
          continue;
        }
        else if (detection.vrel_rad < vrel_rad_threshold)
        {
          continue;
        }
        else if (detection.el_ang < el_ang_threshold)
        {
          continue;
        }
        else if (detection.rcs0 < rcs_threshold)
        {
          continue;
        }
        filtered_ars.detections.push_back(detection);
      }
      return filtered_ars;
    }



// // SOLUTION 2: Via Lambda Functions
// // LAMDA - Filter Template

// radar_msgs::msg::RadarPacket main_filter(
//   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, 
//   const std::function<bool (const radar_msgs::msg::RadarDetection)> &filter_type)
//   {
//     radar_msgs::msg::RadarPacket main_filtered_ars;
//     for (auto detection : unfiltered_ars->detections)
//     {
//       if(filter_type(detection))
//       {
//         // Push filtered point/detection into an array
//         main_filtered_ars.detections.push_back(detection);
//       }
//     }
//     return main_filtered_ars;
//   }

// // LAMBDA - SNR Filter
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::snr_filter(
//   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double snr_threshold)
// {
//   auto filter_type=[snr_threshold](const radar_msgs::msg::RadarDetection test_detection)->bool {
//     return test_detection.snr < snr_threshold;
//   };

//   radar_msgs::msg::RadarPacket filtered_ars = main_filter(unfiltered_ars, filter_type);
//   return filtered_ars;

// }

// // LAMBDA - Azimuth Angle Filter
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::azimuth_angle_filter(
//   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double AzAng0_threshold)
// {
//   auto filter_type=[AzAng0_threshold](const radar_msgs::msg::RadarDetection test_detection)->bool {
//     return test_detection.az_ang0 < AzAng0_threshold;
//   };

//   radar_msgs::msg::RadarPacket filtered_ars = main_filter(unfiltered_ars, filter_type);
//   return filtered_ars;

// }

// // LAMBDA - Range Filter
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::range_filter(
//   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double range_threshold)
// {
//   auto filter_type=[range_threshold](const radar_msgs::msg::RadarDetection test_detection)->bool {
//     return test_detection.range < range_threshold;
//   };

//   radar_msgs::msg::RadarPacket filtered_ars = main_filter(unfiltered_ars, filter_type);
//   return filtered_ars;

// }

// // LAMBDA - Relative Radial Velocity
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::vrel_rad_filter(
//     const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double vrel_rad_threshold)
// {
//   auto filter_type=[vrel_rad_threshold](const radar_msgs::msg::RadarDetection test_detection)->bool {
//     return test_detection.snr < vrel_rad_threshold;
//   };

//   radar_msgs::msg::RadarPacket filtered_ars = main_filter(unfiltered_ars, filter_type);
//   return filtered_ars;
// }

// // LAMBDA - Elevation Angle
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::el_ang_filter(
//   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double el_ang_threshold)
// {
//   auto filter_type=[el_ang_threshold](const radar_msgs::msg::RadarDetection test_detection)->bool {
//     return test_detection.snr < el_ang_threshold;
//   };

//   radar_msgs::msg::RadarPacket filtered_ars = main_filter(unfiltered_ars, filter_type);
//   return filtered_ars;
// }

// // LAMBDA - Radar Cross Section
// radar_msgs::msg::RadarPacket ARSPointCloudFilter::rcs_filter(
//     const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double rcs_threshold)
// {
//   auto filter_type=[rcs_threshold](const radar_msgs::msg::RadarDetection test_detection)->bool {
//     return test_detection.snr < rcs_threshold;
//   };

//   radar_msgs::msg::RadarPacket filtered_ars = main_filter(unfiltered_ars, filter_type);
//   return filtered_ars;
// }






// // ------------------------------
// // ORIGINAL IMPLEMENTATION
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
//   const radar_msgs::msg::RadarPacket::SharedPtr unfiltered_ars, double el_ang_threshold)
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

} // namespace filtering