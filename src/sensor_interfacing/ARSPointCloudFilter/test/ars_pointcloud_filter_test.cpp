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

  // Fake detection Data 1
  msg_detection->vrel_rad = 100.0;
  msg_detection->az_ang0 = 60.0;
  msg_detection->el_ang = 20.0;
  msg_detection->rcs0 = 5.0;
  msg_detection->snr = 100.0;
  msg_detection->range = 90.0;

  msg->detections.push_back(*msg_detection);

  // Fake detection Data 2
  msg_detection->vrel_rad = 100.0;
  msg_detection->az_ang0 = 20.0;
  msg_detection->el_ang = 20.0;
  msg_detection->rcs0 = 5.0;
  msg_detection->snr = 300.0;
  msg_detection->range = 90.0;

  msg->detections.push_back(*msg_detection);
  
  auto test_packet = pointcloudfilter.point_filter(msg, 200, -9999.99, -9999.99, -9999.99, -9999.99, -9999.99);
  
  // Fake detection data 1 should be removed and fake detection data 2 should be at index 0
  EXPECT_EQ(300, test_packet.detections[0].snr);

  test_packet = pointcloudfilter.point_filter(msg, 100, 40, -9999.99, -9999.99, -9999.99, -9999.99);
  // Fake detection data 1 should be left
  EXPECT_EQ(100, test_packet.detections[0].snr);
  EXPECT_EQ(60, test_packet.detections[0].az_ang0);

  // Fake detection data 2 should not exist, so all values should return 0
  EXPECT_EQ(0, test_packet.detections[1].snr);
  EXPECT_EQ(0, test_packet.detections[1].az_ang0);
}

TEST(ARSPointCloudFilterTest, CheckScanType)
{
  filtering::ARSPointCloudFilter pointcloudfilter;

  // Near Scan message
  auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();

  msg->event_id = 3;
  msg->timestamp = 2;
  msg->measurement_counter = 1;
  msg->vambig = 3.0;
  msg->center_frequency = 6.0;

  // Far Scan message
  auto diff_msg = std::make_shared<radar_msgs::msg::RadarPacket>();

  diff_msg->event_id = 1;
  diff_msg->timestamp = 2;
  diff_msg->measurement_counter = 1;
  diff_msg->vambig = 3.0;
  diff_msg->center_frequency = 6.0;

  auto scan_type_0 = pointcloudfilter.check_scan_type(msg);
  auto scan_type_1 = pointcloudfilter.check_scan_type(diff_msg);

  EXPECT_EQ(0, scan_type_0);
  EXPECT_EQ(1, scan_type_1);
}


// TEST(ARSPointCloudFilterTest, ResetScanState)
// {
//   // Need getters and setters to access the private variables in this test
// }

// TEST(ARSPointCloudFilterTest, CommonScanFilter)
// {
//   filtering::ARSPointCloudFilter pointcloudfilter;
//   auto msg = std::make_shared<radar_msgs::msg::RadarPacket>();

//   // make a fake publish packet
//   // make a fake parameter list
//   // make a fake message

//   // verify with buffer packet thats in the member variables
//   // one case for each branch

//   // Near Scan message


// }