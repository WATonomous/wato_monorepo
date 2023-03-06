#include <memory>
#include <string>
#include <tuple>

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"

#include "ars_pointcloud_filter.hpp"

// Unit test cases 
// 1. Single packet (near)
// 2. Single packet (far)
// 3. Multiple packets with same timestamps (near)
// 4. Multiple packets with different timestamps (near)

TEST(ARSPointCloudFilterTest, PointFilter)
{
  filtering::ARSPointCloudFilter pointcloudfilter;
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();
  auto msg_detection = std::make_shared<radar_msgs::msg::RadarDetection>();

  msg->event_id = 3;
  msg->timestamp = 2;
  msg->measurement_counter = 1;
  msg->vambig = 3.0;
  msg->center_frequency = 6.0;

  msg_detection->pos_x = 1.0;
  msg_detection->pos_y = 3.0;
  msg_detection->pos_z = 4.0;
  msg_detection->vrel_rad = 100.0;
  msg_detection->az_ang0 = 60.0;
  msg_detection->az_ang1 = 50.0;
  msg_detection->el_ang = 20.0;
  msg_detection->rcs0 = 5.0;
  msg_detection->rcs1 = 10.0;
  msg_detection->range_var = 600.0;
  msg_detection->vrel_rad_var = 175.0;
  msg_detection->az_ang_var0 = 12.0;
  msg_detection->az_ang_var1 = 17.0;
  msg_detection->el_ang_var = 19.0;
  msg_detection->snr = 100.0;
  msg_detection->range = 90.0;
  msg_detection->prob0 = 0.0;
  msg_detection->prob1 = 1.0;
  msg_detection->pdh0 = 255.0;

  msg->detections.push_back(*msg_detection);

  
  msg_detection->pos_x = 1.0;
  msg_detection->pos_y = 3.0;
  msg_detection->pos_z = 4.0;
  msg_detection->vrel_rad = 100.0;
  msg_detection->az_ang0 = 60.0;
  msg_detection->az_ang1 = 50.0;
  msg_detection->el_ang = 20.0;
  msg_detection->rcs0 = 5.0;
  msg_detection->rcs1 = 10.0;
  msg_detection->range_var = 600.0;
  msg_detection->vrel_rad_var = 175.0;
  msg_detection->az_ang_var0 = 12.0;
  msg_detection->az_ang_var1 = 17.0;
  msg_detection->el_ang_var = 19.0;
  msg_detection->snr = 300.0;
  msg_detection->range = 90.0;
  msg_detection->prob0 = 0.0;
  msg_detection->prob1 = 1.0;
  msg_detection->pdh0 = 255.0;

  msg->detections.push_back(*msg_detection);
  
  auto test_packet = pointcloudfilter.point_filter(msg, 200, -9999.99, -9999.99, -9999.99, -9999.99, -9999.99);
  
  EXPECT_EQ(300, test_packet.detections[0].snr);
}
